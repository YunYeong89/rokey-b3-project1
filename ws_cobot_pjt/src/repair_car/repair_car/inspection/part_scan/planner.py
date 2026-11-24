"""Raster paths over a part along fixed probe axes (robot.yaml real_scan.axes: bonnet and bumper from above, the
doors from either side), as defect_scan does on the block, from measurable line segments. Per axis the outermost
surface (HeightMap) and its measurable cells (real_scan.measurable): off the edges and openings, not steeper than
tilt_deg, on the outer skin where skin_mm is set (not recessed behind it: the door's lower edge turning under), tool
clear and reachable. Then per pass, horizontal (rows along the grid's u) and vertical (along v), every 0.5 mm line
followed as a straight move would go: IK from the last cell, a cell dropped where it fails or the joints jump; the
runs of what is left are the measurable segments (`maps`, drawn by report.plan). Lines every pitch are selected and
their segments joined into one segment per connected measurable region, the probe in contact throughout, riding the
slopes by force control: line by line (serpentine), a line broken by a gap continues as a stripe on its side, the
rest reached around it; every join a straight move over measurable cells, or the shortest way round them
straightened. The tool keeps the axis throughout. A segment is kept from its first to its last point that can be
approached and left: out along the axis to the standoff point, the tool turned down there, straight up to the travel
height (tool and arm clear, within reach, at samples of each leg). One path per pass (`sweeps`: horizontal, then
vertical, per axis): its segments from the longest on, each time the nearest end of the rest, the probe lifted only
between them, straight across at the standoff where that is clear and within reach, else over the travel height.
`paths`: the segments as (BASE mm surface points, axis)."""
from collections import deque

import numpy as np

from repair_car.robot_control.rotations import align, unit
from repair_car.pointcloud.cloud import HeightMap, max_filter


def per_cell(points, cell, test):
    """test(i) for one point per `cell` cube, spread to the points of that cube."""
    _, first, index = np.unique(np.floor(points / cell).astype(np.int64), axis=0, return_index=True,
                                return_inverse=True)
    return np.array([bool(test(i)) for i in first], bool)[index.ravel()]


def on_mask(mask, start, goal):
    """Whether the straight segment start -> goal (grid cells, two samples per cell) stays on the mask cells."""
    cells = np.rint(np.linspace(start, goal, 2 * int(np.ceil(np.abs(goal - start).max())) + 1)).astype(int)
    return bool(mask[cells[:, 0], cells[:, 1]].all())


def neighbours(mask, cell):
    a, b = cell
    return [near for near in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1))
            if 0 <= near[0] < mask.shape[0] and 0 <= near[1] < mask.shape[1] and mask[near]]


def components(mask):
    """Labels 1.. of the 4-connected pieces of the mask cells (0 off it)."""
    labels, count = np.zeros(mask.shape, int), 0
    for start in zip(*np.nonzero(mask)):
        if labels[start]:
            continue
        count += 1
        labels[start], queue = count, deque([start])
        while queue:
            for near in neighbours(mask, queue.popleft()):
                if not labels[near]:
                    labels[near] = count
                    queue.append(near)
    return labels


def search(mask, start, goal):
    """Shortest 4-connected route start -> goal over the mask cells (breadth first; no corner cutting)."""
    start, goal = tuple(start), tuple(goal)
    previous, queue = {start: None}, deque([start])
    while goal not in previous:
        for near in neighbours(mask, cell := queue.popleft()):
            if near not in previous:
                previous[near] = cell
                queue.append(near)
    track, cell = [], goal
    while cell is not None:
        track.append(cell)
        cell = previous[cell]
    return np.array(track[::-1])


def corners(mask, start, goal):
    """Corner cells of a way start -> goal over the mask (one connected region): straight when the segment stays on
    it, else the shortest route straightened (from each corner on as far as the line of sight reaches)."""
    if on_mask(mask, start, goal):
        return [start, goal]
    track = search(mask, start, goal)
    points, at = [track[0]], 0
    while at < len(track) - 1:
        reach = at + 1
        while reach + 1 < len(track) and on_mask(mask, track[at], track[reach + 1]):
            reach += 1
        points.append(track[reach])
        at = reach
    return points


def densify(points):
    """The polyline through `points` (cells) every half cell at most, collinear between its corners."""
    legs = [np.linspace(a, b, int(np.ceil(2 * np.abs(b - a).max())) + 1)[1:] for a, b in zip(points[:-1], points[1:])]
    return np.concatenate([np.asarray(points[:1], float), *legs])


def stripes(mask, lines, shortest):
    """Measurable segments on the selected lines (columns `lines` of the mask): runs of consecutive cells at least
    `shortest` cells long, as (line number, (n, 2) cells)."""
    runs = []
    for k, line in enumerate(lines):
        cells = np.nonzero(mask[:, line])[0]
        runs += [(k, np.c_[c, np.full(len(c), line)])
                 for c in np.split(cells, np.nonzero(np.diff(cells) > 1)[0] + 1) if len(c) >= shortest]
    return runs


def raster(mask, runs):
    """One path through all the segments of one connected region: from the first on, each time the segment on the
    next line overlapping the last one (the line direction turned when only the other side has one), else the
    nearest one left; either way round, whichever end is nearer; joined over the region (corners). Between adjacent
    lines the join is straight when it stays within one line spacing of the region: no backtrack along a longer
    line, so no reversal (a stop) at the line ends."""
    pending = list(runs)
    k, cells = pending.pop(0)
    parts, step = [cells], 1
    lines = np.unique([c[0, 1] for _, c in runs])
    loose = mask.copy()
    for _ in range(int(np.diff(lines).min()) if len(lines) > 1 else 1):
        grown = loose.copy()
        grown[1:] |= loose[:-1]
        grown[:-1] |= loose[1:]
        grown[:, 1:] |= loose[:, :-1]
        grown[:, :-1] |= loose[:, 1:]
        loose = grown

    def overlapping(line, low, high):
        return [n for n, (j, c) in enumerate(pending)
                if j == line and c[:, 0].min() <= high + 1 and c[:, 0].max() >= low - 1]

    while pending:
        low, high = cells[:, 0].min(), cells[:, 0].max()
        choice = overlapping(k + step, low, high)
        if not choice:
            step = -step
            choice = overlapping(k + step, low, high) or range(len(pending))
        n, end = min(((n, end) for n in choice for end in (0, -1)),
                     key=lambda pick: np.abs(pending[pick[0]][1][pick[1]] - cells[-1]).sum())
        last, line = cells[-1], k
        k, cells = pending.pop(n)
        cells = cells[::-1] if end else cells
        straight = abs(k - line) == 1 and on_mask(loose, last, cells[0])
        parts += [densify([last, cells[0]] if straight else corners(mask, last, cells[0]))[1:-1], cells]
    return np.concatenate(parts).astype(float)



def sweep(mask, runs, *, strict=False):
    """One path through all the segments of one connected region line after line in the lines' order, never back
    (raster may turn back to reach a segment): on each line its segments in order along it, every other line the
    other way round (serpentine). Joined straight between adjacent lines when that stays within one line spacing of
    the region, else around over it (corners), also between the segments of one line."""
    columns = np.unique([c[0, 1] for _, c in runs])
    loose = mask.copy()
    growth = int(np.diff(columns).min()) if len(columns) > 1 else 1
    for _ in range(0 if strict else growth):
        grown = loose.copy()
        grown[1:] |= loose[:-1]
        grown[:-1] |= loose[1:]
        grown[:, 1:] |= loose[:, :-1]
        grown[:, :-1] |= loose[:, 1:]
        loose = grown
    ordered = []
    for n, line in enumerate(sorted({k for k, _ in runs})):
        own = sorted((c for k, c in runs if k == line), key=lambda c: c[0, 0])
        ordered += [(line, c) for c in (own if n % 2 == 0 else [c[::-1] for c in own[::-1]])]
    (k, cells), parts = ordered[0], [ordered[0][1]]
    for line, following in ordered[1:]:
        last = cells[-1]
        straight = abs(line - k) == 1 and on_mask(loose, last, following[0])
        parts += [densify([last, following[0]] if straight else corners(mask, last, following[0]))[1:-1], following]
        k, cells = line, following
    return np.concatenate(parts).astype(float)


class PartPlan:
    def __init__(self, cloud, axes, clearance, reach, scan, radius, travel_z):  # radius: probe ball, mm
        self.clearance, self.reach, self.scan, self.travel = clearance, reach, scan, travel_z
        self.paths, self.sweeps, self.count = [], [], dict(axes=0, points=0, clear=0, reachable=0)
        self.grids, self.maps = [], []  # (axis, HeightMap); (axis, HeightMap, along, measurable cells, paths)
        margin = scan['edge_margin_mm'].get(cloud.part, scan['edge_margin_mm']['default'])
        centre = (cloud.points.min(0) + cloud.points.max(0)) / 2
        for axis in np.asarray(axes, float):
            # From a side only that side's half: through a door window the other door's inside would show.
            points = cloud.points if abs(axis[2]) > .9 else cloud.points[(cloud.points - centre) @ axis > 0]
            grid = HeightMap(points, axis, scan['step_mm'], radius, margin)
            self.count['axes'] += 1
            self.grids.append((axis, grid))
            cells = self.measurable(grid, cloud.part)
            for along in (0, 1):  # horizontal pass, then the vertical one
                valid = self.continuous(grid, cells, along)
                paths = self.passes(valid, grid.step, along)
                self.maps.append((axis, grid, along, valid, paths))
                segments = [kept for kept in (self.trim(grid.positions(path[:, 0], path[:, 1]), axis) for path in paths)
                            if kept is not None]
                if segments:  # (segments, axis, straight lift between consecutive segments)
                    segments = self.order(segments)
                    self.sweeps.append((segments, axis, [self.hop(a[-1], b[0], axis)
                                                         for a, b in zip(segments[:-1], segments[1:])]))
                    self.paths += [(segment, axis) for segment in segments]

    def accessible(self, positions, axis):
        ok = per_cell(positions, self.scan['check_mm'], lambda k: self.clearance.clear(positions[k], axis))
        clear = np.nonzero(ok)[0]
        if len(clear):
            ok[clear] = per_cell(positions[clear], self.scan['ik_mm'],
                                 lambda k: self.reach.reachable(positions[clear[k]], axis))
        for key, value in (('points', len(positions)), ('clear', len(clear)), ('reachable', ok.sum())):
            self.count[key] += int(value)
        return ok

    def measurable(self, grid, part):
        """Cells off the edges and openings, not steeper than tilt_deg, on the outer skin (at most skin_mm below the
        outermost surface within skin_window_mm, where skin_mm is set for the part), tool clear and reachable."""
        m = self.scan['measurable']
        mask = grid.inner & (grid.tilt <= m['tilt_deg'].get(part, m['tilt_deg']['default']))
        skin = m['skin_mm'].get(part)
        if skin is not None:
            outer = grid.height >= max_filter(grid.height, int(np.ceil(m['skin_window_mm'] / grid.step))) - skin
            # The skin's own edge keeps the edge margin too: a row on the door's last skin cells (its lower edge
            # turning under) let the pressed ball slide off into the recess and stay trapped under the lip.
            margin = self.scan['edge_margin_mm'].get(part, self.scan['edge_margin_mm']['default'])
            mask &= max_filter(np.where(outer, 0., 1.), int(np.ceil(margin / grid.step)), outside=1.) <= 0
        i, j = np.nonzero(mask)
        if len(i):
            mask[i, j] = self.accessible(grid.positions(i, j), grid.axis)
        return mask

    def continuous(self, grid, cells, along):
        """The cells a straight move along the pass's lines gets through: IK from the last cell of the line (from
        HOME at its start, after a gap or a break), a cell dropped where it fails or a joint moves more than
        joint_jump_deg."""
        chain, rotation = self.reach.chain, align(self.reach.frame, grid.axis, np.pi)
        jump = np.radians(self.scan['measurable']['joint_jump_deg'])
        valid, lines = np.zeros_like(cells), cells if along == 0 else cells.T
        for line in np.nonzero(lines.any(0))[0]:
            run = np.nonzero(lines[:, line])[0]
            i, j = (run, np.full(len(run), line)) if along == 0 else (np.full(len(run), line), run)
            tips, q = grid.positions(i, j) / 1000, None
            for n in range(len(run)):
                if n and run[n] != run[n - 1] + 1:
                    q = None
                following = chain.solve(tips[n], rotation, self.reach.home if q is None else q)
                if following is not None and (q is None or np.abs(following - q).max() <= jump):
                    valid[i[n], j[n]], q = True, following
                else:
                    q = None
        return valid

    def passes(self, valid, step, along, join=raster):
        """Lines along u (`along` 0: one per grid column) or v (1: one per grid row) every pitch_mm, their
        measurable segments (min_segment_mm) rastered (`join`) into one path per connected measurable region. Paths: (n, 2)
        cells (i, j), fractional on the joins."""
        m = self.scan['measurable']
        mask = valid if along == 0 else valid.T  # [along the line, line]
        lines = np.nonzero(mask.any(0))[0]
        lines = lines[(lines - lines[:1]) % max(1, int(round(self.scan['pitch_mm'] / step))) == 0]  # pitch apart
        labels, runs = components(mask), stripes(mask, lines, int(np.ceil(m['min_segment_mm'] / step)))
        paths = []
        for label in range(1, labels.max() + 1):
            own = [run for run in runs if labels[tuple(run[1][0])] == label]
            if own:
                paths.append(join(labels == label, own))
        return [path[:, ::-1] if along else path for path in paths]

    def on_measurable(self, points, tolerance=1.5):
        """Which points (BASE mm) lie on a measurable cell of either pass of an axis, on the outer surface there
        (within `tolerance` of its height): the part as the probe can touch it, for the RViz reference cloud."""
        keep = np.zeros(len(points), bool)
        for (axis, grid, _, horizontal, _), (_, _, _, vertical, _) in zip(self.maps[::2], self.maps[1::2]):
            cells = np.floor((np.c_[points @ grid.u, points @ grid.v] - grid.origin) / grid.step).astype(int)
            inside = np.nonzero(np.all((cells >= 0) & (cells < grid.height.shape), axis=1))[0]
            i, j = cells[inside].T
            near = np.abs(points[inside] @ axis - grid.height[i, j]) < tolerance
            keep[inside[(horizontal | vertical)[i, j] & near]] = True
        return keep

    def transit(self, tip, axis):
        """Out along the axis to the standoff, the tool turned down there, straight up to the travel height."""
        out, up = tip + self.scan['standoff_mm'] * axis, np.array([0., 0., 1.])
        z = max(out[2], self.travel)
        margin = self.scan.get('travel_margin_mm', 0.)
        if margin:
            high = np.array([[*out[:2], z]])
            if not self.reach.within_outer_reach(high, up, margin)[0]:
                return False
        poses = [(tip + d * axis, axis) for d in np.linspace(10., self.scan['standoff_mm'], 4)]
        poses += [(out, unit((1 - t) * axis + t * up)) for t in (.25, .5, .75, 1.)]
        poses += [(np.r_[out[:2], height], up) for height in np.linspace(out[2], z, 4)]
        return all(self.clearance.clear(p, a) and self.reach.reachable(p, a, transfer=True) for p, a in poses)

    def trim(self, positions, axis, every=4):
        """Positions from the first to the last point (every few) with a clear transit, or None."""
        first = next((i for i in range(0, len(positions), every) if self.transit(positions[i], axis)), None)
        if first is None:
            return None
        last = next(i for i in [*range(len(positions) - 1, first, -every), first] if self.transit(positions[i], axis))
        kept = positions[first:last + 1]
        length = np.linalg.norm(np.diff(kept, axis=0), axis=1).sum()
        return kept if len(kept) >= 2 and length >= self.scan['min_path_mm'] else None

    @staticmethod
    def order(segments):
        """The longest segment first, then each time the one with the nearest end (reversed when it is its last)."""
        pending = sorted(segments, key=lambda points: np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        ordered = [pending.pop()]
        while pending:
            end = ordered[-1][-1]
            n, last = min(((n, last) for n in range(len(pending)) for last in (0, -1)),
                          key=lambda pick: np.linalg.norm(pending[pick[0]][pick[1]] - end))
            segment = pending.pop(n)
            ordered.append(segment[::-1] if last else segment)
        return ordered

    def hop(self, end, start, axis):
        """The straight move between the standoff points off a segment's end and the next one's start (tool on the
        axis) is clear and within reach."""
        a, b = end + self.scan['standoff_mm'] * axis, start + self.scan['standoff_mm'] * axis
        count = int(np.ceil(np.linalg.norm(b - a) / self.scan['check_mm'])) + 1
        return all(self.clearance.clear(p, axis) and self.reach.reachable(p, axis, transfer=True)
                   for p in np.linspace(a, b, count))

    def summary(self, speed_mm_s):
        c, length = self.count, sum(np.linalg.norm(np.diff(p, axis=0), axis=1).sum() for p, _ in self.paths)
        share = lambda key: c[key] / max(c['points'], 1)
        measurable = ', '.join(f'{"hv"[along]} {valid.sum()}' for _, _, along, valid, _ in self.maps)
        return (f'{c["axes"]} axes, {c["points"]} candidate cells: tool clear {share("clear"):.0%}, clear and '
                f'reachable {share("reachable"):.0%}; measurable along the lines {measurable}; {len(self.sweeps)} '
                f'paths ({len(self.paths)} segments, lifted between), {length:.0f} mm, '
                f'~{length / speed_mm_s / 60 + .75 * len(self.paths):.0f} min')
