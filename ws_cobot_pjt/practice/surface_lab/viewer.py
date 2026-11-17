"""Local Pygame dashboard: physical probe, contact signals and adjustable experiment parameters."""
from dataclasses import dataclass
from math import degrees, radians
from pathlib import Path

import pygame
from pymunk import Vec2d

from .geometry import Terrain, limit, normal_angle
from .config import Settings
from .recording import Recording


BG = (13, 20, 32)
PANEL = (22, 33, 49)
INK = (228, 237, 249)
MUTED = (143, 163, 187)
CYAN = (66, 217, 224)
RED = (255, 108, 127)
GREEN = (110, 229, 159)
GOLD = (245, 195, 103)
BLUE = (113, 162, 255)


@dataclass
class Button:
    rect: pygame.Rect
    label: str
    callback: object
    active: bool = False


class Slider:
    def __init__(self, label, low, high, get, set_value, fmt):
        self.label, self.low, self.high = label, low, high
        self.get, self.set_value, self.fmt = get, set_value, fmt
        self.rect = pygame.Rect(0, 0, 1, 1)

    def move(self, x):
        fraction = limit((x - self.rect.x) / self.rect.width, 0, 1)
        self.set_value(self.low + fraction * (self.high - self.low))


class Application:
    def __init__(self, experiment, paused=False):
        self.exp, self.paused = experiment, paused
        pygame.display.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((1440, 930), pygame.RESIZABLE)
        pygame.display.set_caption('2D Probe Lab | Hybrid / Admittance')
        candidates = ['/System/Library/Fonts/AppleSDGothicNeo.ttc',
                      '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']
        font = next((name for name in candidates if Path(name).is_file()), None)
        self.fonts = {size: pygame.font.Font(font, size) for size in (13, 15, 17, 20, 26, 30)}
        self.clock = pygame.time.Clock()
        self.message = 'Space: brake / resume   R: reset   H / A: controller   1..4: scene   F1/F2: model   S: save   P: load pulse'
        self.buttons, self.dragging = [], None
        self.panel = 'environment'
        s = self.exp.settings
        self.environment_sliders = [
            Slider('Target normal force', .3, 4.0, lambda: s.target_force,
                   lambda v: setattr(s, 'target_force', v), '{:.2f} N'),
            Slider('Tangential speed', 1, 40, lambda: s.feed * 1000,
                   lambda v: setattr(s, 'feed', v / 1000), '{:.1f} mm/s'),
            Slider('Playback speed', .5, 8, lambda: s.time_scale,
                   lambda v: setattr(s, 'time_scale', v), '{:.1f}x'),
            Slider('Friction (physics)', 0, .6, lambda: s.friction,
                   lambda v: self.exp.physics.set_friction(v), '{:.2f}'),
            Slider('Sensor noise sigma', 0, .15, lambda: s.noise,
                   lambda v: setattr(s, 'noise', v), '{:.3f} N'),
        ]
        rotation = Slider('Max tool angular speed', 10, 150, lambda: degrees(s.max_angular_speed),
                          lambda v: setattr(s, 'max_angular_speed', radians(v)), '{:.0f} deg/s')
        self.hybrid_sliders = [
            Slider('Normal force Kp', 0, 2, lambda: s.hybrid_kp,
                   lambda v: setattr(s, 'hybrid_kp', v), '{:.2f}'),
            Slider('Normal force Ki', 0, 2, lambda: s.hybrid_ki,
                   lambda v: setattr(s, 'hybrid_ki', v), '{:.2f} /s'),
            Slider('Normal damping', 5, 100, lambda: s.normal_damping,
                   lambda v: setattr(s, 'normal_damping', v), '{:.1f} Ns/m'),
            Slider('Tangential servo stiffness', 500, 5000, lambda: s.position_stiffness,
                   lambda v: setattr(s, 'position_stiffness', v), '{:.0f} N/m'), rotation,
        ]
        self.admittance_sliders = [
            Slider('Virtual mass M', .1, 20, lambda: s.admittance_mass,
                   lambda v: setattr(s, 'admittance_mass', v), '{:.2f} kg'),
            Slider('Virtual damping D', 50, 2000, lambda: s.admittance_damping,
                   lambda v: setattr(s, 'admittance_damping', v), '{:.0f} Ns/m'),
            Slider('Inner servo stiffness', 500, 5000, lambda: s.position_stiffness,
                   lambda v: setattr(s, 'position_stiffness', v), '{:.0f} N/m'),
            Slider('Inner servo damping', 10, 200, lambda: s.velocity_damping,
                   lambda v: setattr(s, 'velocity_damping', v), '{:.1f} Ns/m'), rotation,
        ]
        self.timing_sliders = [
            Slider('Force filter time constant', 0, 150, lambda: s.sensor_tau * 1000,
                   lambda v: setattr(s, 'sensor_tau', v / 1000), '{:.0f} ms'),
            Slider('Feedback delay', 0, 100, lambda: s.feedback_delay * 1000,
                   lambda v: setattr(s, 'feedback_delay', v / 1000), '{:.0f} ms'),
            Slider('Command delay', 0, 80, lambda: s.command_delay * 1000,
                   lambda v: setattr(s, 'command_delay', v / 1000), '{:.0f} ms'),
            Slider('Transport jitter', 0, 30, lambda: s.jitter * 1000,
                   lambda v: setattr(s, 'jitter', v / 1000), '{:.0f} ms'),
            Slider('Missed feedback updates', 0, 20, lambda: s.dropout * 100,
                   lambda v: setattr(s, 'dropout', v / 100), '{:.1f} %'),
        ]

    @property
    def sliders(self):
        if self.panel == 'environment':
            return self.environment_sliders
        if self.panel == 'timing':
            return self.timing_sliders
        return self.hybrid_sliders if self.exp.settings.controller == 'hybrid' else self.admittance_sliders

    def text(self, value, position, size=15, color=INK):
        self.screen.blit(self.fonts[size].render(str(value), True, color), position)

    def change(self, name, value):
        setattr(self.exp.settings, name, value)
        self.exp.reset()
        self.message = 'New run. Scene/controller changes reset the experiment; use S to save before switching.'

    def save(self):
        directory = Recording.save(self.exp)
        pygame.image.save(self.screen, str(directory / 'screen.png'))
        self.message = f'Saved: {directory.name} / signals.csv, points.csv, summary.json, screen.png'
        print('Saved:', directory, flush=True)

    def toggle_pause(self):
        if self.paused:
            self.exp.resume()
            self.paused = False
        elif not self.exp.stopped:
            self.exp.pause()  # Continue physics until brake acknowledgement.

    def edit_slider(self, slider, x):
        if self.panel == 'gains' and self.exp.settings.preset == 'robot' and not (self.paused or self.exp.stopped):
            self.message = 'Pause and wait for acknowledgement before changing robot-model gains.'
            return
        slider.move(x)

    def change_preset(self, name):
        keep = {key: getattr(self.exp.settings, key) for key in ('scene', 'controller', 'feed', 'target_force')}
        preset = Settings.for_preset(name)
        for key, value in vars(preset).items():
            setattr(self.exp.settings, key, value)
        for key, value in keep.items():
            setattr(self.exp.settings, key, value)
        self.exp.reset()
        self.message = 'New trial. Robot rates are matched; transport and Cartesian dynamics are configurable assumptions.'

    def layout(self):
        width, height = self.screen.get_size()
        self.world = pygame.Rect(20, 148, width - 400, height - 350)
        self.side = pygame.Rect(width - 360, 148, 340, height - 178)
        self.graph_area = pygame.Rect(20, self.world.bottom + 16, self.world.width, 150)
        self.buttons = []
        options = [('H  Hybrid', 'controller', 'hybrid', 20, 135),
                   ('A  Admittance', 'controller', 'admittance', 164, 165)]
        for label, name, value, x, w in options:
            self.buttons.append(Button(pygame.Rect(x, 87, w, 38), label,
                                       lambda n=name, v=value: self.change(n, v),
                                       getattr(self.exp.settings, name) == value))
        for index, scene in enumerate(Terrain.names):
            self.buttons.append(Button(pygame.Rect(353 + index * 116, 87, 108, 38),
                                       f'{index + 1}  {scene.title()}', lambda v=scene: self.change('scene', v),
                                       self.exp.settings.scene == scene))
        for index, (label, function) in enumerate([
                ('Resume' if self.paused else 'Pause', self.toggle_pause),
                ('Reset', self.exp.reset), ('Save', self.save)]):
            self.buttons.append(Button(pygame.Rect(width - 360 + index * 113, 87, 106, 38), label, function))
        for index, (label, panel) in enumerate([('Environment', 'environment'), ('Gains', 'gains'), ('I/O delay', 'timing')]):
            self.buttons.append(Button(pygame.Rect(self.side.x + 20 + index * 102, self.side.y + 42, 96, 27),
                                       label, lambda v=panel: setattr(self, 'panel', v), self.panel == panel))
        self.buttons.append(Button(pygame.Rect(self.world.right - 235, self.world.y + 10, 218, 27),
                                   f'F1/F2 model: {self.exp.settings.preset.upper()}',
                                   lambda: self.change_preset('ideal' if self.exp.settings.preset == 'robot' else 'robot')))
        for index, slider in enumerate(self.sliders):
            slider.rect = pygame.Rect(self.side.x + 20, self.side.y + 110 + index * 56, self.side.width - 40, 12)
        # Fit all shapes with enough room for a shaft above the highest point.
        self.xmin, self.xmax = -.035, .735
        self.ymin, self.ymax = .055, .465
        self.scale = min((self.world.width - 42) / (self.xmax - self.xmin),
                         (self.world.height - 55) / (self.ymax - self.ymin))
        self.offset = Vec2d(self.world.centerx - (self.xmin + self.xmax) / 2 * self.scale,
                            self.world.centery + (self.ymin + self.ymax) / 2 * self.scale + 8)

    def pixel(self, point):
        return round(self.offset.x + point[0] * self.scale), round(self.offset.y - point[1] * self.scale)

    def arrow(self, start, vector, color, pixels, label=None, width=3, label_offset=(6, -12)):
        if vector.length < 1e-6:
            return
        delta = Vec2d(vector.x, -vector.y) * pixels
        if delta.length > 145:
            delta *= 145 / delta.length
        if delta.length < 3:
            return
        begin = Vec2d(*self.pixel(start))
        end = begin + delta
        pygame.draw.line(self.screen, color, begin, end, width)
        direction = delta.normalized()
        wing = direction.perpendicular() * 4
        pygame.draw.polygon(self.screen, color, [end, end - 10 * direction + wing, end - 10 * direction - wing])
        if label:
            self.text(label, end + Vec2d(*label_offset), 13, color)

    def draw_world(self):
        pygame.draw.rect(self.screen, PANEL, self.world, border_radius=12)
        self.screen.set_clip(self.world.inflate(-2, -2))
        for k in range(-1, 16):
            x = k * .05
            pygame.draw.line(self.screen, (32, 46, 64), self.pixel((x, .04)), self.pixel((x, .48)))
        for k in range(1, 10):
            y = k * .05
            pygame.draw.line(self.screen, (32, 46, 64), self.pixel((-.04, y)), self.pixel((.74, y)))
        path = [self.pixel(point) for point in self.exp.terrain.points]
        bottom = self.pixel((0, self.ymin - 1))[1]
        pygame.draw.polygon(self.screen, (40, 49, 60), [*path, (path[-1][0], bottom), (path[0][0], bottom)])
        pygame.draw.lines(self.screen, GOLD, False, path, 3)
        for point in self.exp.points:
            pygame.draw.circle(self.screen, (222, 231, 248), self.pixel(point), 2)
        p = self.exp.physics
        centre, head = self.pixel(p.body.position), self.pixel(p.head)
        pygame.draw.line(self.screen, (79, 105, 132), head, centre, 12)
        pygame.draw.line(self.screen, (176, 195, 213), head, centre, 5)
        axis = p.axis
        cap_left, cap_right = p.head - .014 * axis.perpendicular(), p.head + .014 * axis.perpendicular()
        pygame.draw.line(self.screen, BLUE, self.pixel(cap_left), self.pixel(cap_right), 11)
        pygame.draw.circle(self.screen, (153, 213, 244), centre, max(2, round(self.exp.settings.tip_radius * self.scale)))
        pygame.draw.circle(self.screen, BG, centre, 3)
        if p.sensor.contact is not None:
            pygame.draw.circle(self.screen, RED, self.pixel(p.sensor.contact), 4)
        # Ground truth is visible only as a diagnostic; controller inputs exclude it.
        _, true_normal = self.exp.terrain.nearest(p.body.position)
        self.arrow(p.body.position, true_normal, (117, 132, 151), 47, 'n true', width=1)
        self.arrow(p.body.position, self.exp.controller.normal, CYAN, 78, 'n estimated')
        self.arrow(p.head, self.exp.sensor.filtered, RED, 40, 'F received')
        self.arrow(p.body.position, p.body.velocity, GREEN, 6500, 'v actual', label_offset=(6, -23))
        self.arrow(p.body.position, self.exp.command.velocity, BLUE, 5000, 'v command', width=2, label_offset=(6, 2))
        self.arrow(p.head, p.actuator_force, GOLD, 27, 'actuator', width=2)
        target = self.pixel(self.exp.command.position)
        pygame.draw.circle(self.screen, BLUE, target, 5, 1)
        self.screen.set_clip(None)
        self.text('X / Y [m]  |  grid 50 mm  |  spherical tip + shaft',
                  (self.world.x + 16, self.world.y + 12), 13, MUTED)
        law = ('Hybrid: tangent position servo + normal force PI / damping' if self.exp.settings.controller == 'hybrid'
               else 'Admittance: M du/dt + D u = Fd - Fn  |  position servo follows integrated velocity')
        self.text(law, (self.world.x + 16, self.world.y + 32), 13, MUTED)
        legend = [('F sensor', RED), ('motion', GREEN), ('n estimated', CYAN),
                  ('n true: debug', (145, 156, 171)), ('actuator force', GOLD)]
        for index, (label, color) in enumerate(legend):
            self.text(label, (self.world.x + 18 + index * 145, self.world.bottom - 26), 13, color)

    def draw_side(self):
        rect, s, c, p = self.side, self.exp.settings, self.exp.controller, self.exp.physics
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=12)
        self.text('EXPERIMENT CONTROLS', (rect.x + 20, rect.y + 16), 17)
        for slider in self.sliders:
            value = slider.get()
            self.text(slider.label, (slider.rect.x, slider.rect.y - 24), 13, MUTED)
            rendered = self.fonts[13].render(slider.fmt.format(value), True, INK)
            self.screen.blit(rendered, (slider.rect.right - rendered.get_width(), slider.rect.y - 24))
            pygame.draw.rect(self.screen, (50, 66, 87), slider.rect, border_radius=6)
            fraction = limit((value - slider.low) / (slider.high - slider.low), 0, 1)
            progress = slider.rect.copy()
            progress.width = max(1, int(slider.rect.width * fraction))
            pygame.draw.rect(self.screen, BLUE, progress, border_radius=6)
            pygame.draw.circle(self.screen, INK, (int(slider.rect.x + fraction * slider.rect.width), slider.rect.centery), 7)
        y = rect.y + 382
        color = RED if c.phase == 'STOPPED' else GREEN if c.phase == 'TRACK' else GOLD
        self.text(c.phase, (rect.x + 20, y), 26, color)
        y += 42
        self.text(f'{c.force:+.3f}', (rect.x + 20, y), 30, color)
        self.text(f'N   target {s.target_force:.2f}', (rect.x + 138, y + 8), 17, MUTED)
        rows = [
            ('Force error', f'{s.target_force - c.force:+.3f} N'),
            ('Fx / Fy received', f'{self.exp.sensor.filtered.x:+.2f} / {self.exp.sensor.filtered.y:+.2f} N'),
            ('Wrist moment', f'{self.exp.sensor.torque:+.4f} Nm'),
            ('Normal direction', f'({c.normal.x:+.3f}, {c.normal.y:+.3f})'),
            ('Tool / normal angle', f'{degrees(p.body.angle):+.1f} / {degrees(normal_angle(c.normal)):+.1f} deg'),
            ('Estimated friction', f'{c.observer.mu:.3f}'),
            ('Measured points', str(len(self.exp.points))),
            ('FT / state / control', f'{s.sensor_hz} / {s.feedback_hz} / {s.control_hz} Hz'),
            ('State age / protection', f'{(self.exp.time-self.exp.latest.sampled_at)*1000:.0f} ms / {"ON" if s.adaptive else "baseline"}'),
            ('Fast / predicted force', f'{c.supervisor.fast:.2f} / {c.supervisor.predicted:.2f} N'),
        ]
        for index, (label, value) in enumerate(rows):
            yy = y + 54 + index * 27
            self.text(label, (rect.x + 20, yy), 13, MUTED)
            rendered = self.fonts[13].render(value, True, INK)
            self.screen.blit(rendered, (rect.right - 20 - rendered.get_width(), yy))

    def graph(self, rect, title, series, minimum, maximum, units):
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=10)
        self.text(title, (rect.x + 12, rect.y + 9), 13, MUTED)
        rows = list(self.exp.history)
        if not rows:
            return
        plot = pygame.Rect(rect.x + 10, rect.y + 36, rect.width - 20, rect.height - 58)
        start = max(0, self.exp.time - 10)
        span = 10
        for getter, color in series:
            points = []
            for row in rows[::max(1, len(rows) // 450)]:
                if row['time_s'] < start:
                    continue
                fraction = limit((getter(row) - minimum) / (maximum - minimum), 0, 1)
                points.append((plot.x + (row['time_s'] - start) / span * plot.width,
                               plot.bottom - fraction * plot.height))
            if len(points) > 1:
                pygame.draw.lines(self.screen, color, False, points, 2)
        self.text(f'{minimum:g} .. {maximum:g} {units}', (rect.x + 10, rect.bottom - 20), 13, MUTED)
        self.text('last 10 s', (rect.right - 70, rect.bottom - 20), 13, MUTED)

    def draw(self):
        self.layout()
        self.screen.fill(BG)
        self.text('2D PROBE / SURFACE FOLLOWING', (22, 18), 26)
        self.text('접촉 물리 · 힘 센서 · 이동 방향 · 법선 추정 · 힘 제어 비교', (23, 53), 15, MUTED)
        state = ('PAUSED' if self.paused else self.exp.controller.phase
                 if self.exp.stopped or self.exp.brake_started is not None else 'RUNNING')
        self.text(f'{state}   t={self.exp.time:6.2f} s   {self.exp.settings.time_scale:.1f}x',
                  (self.screen.get_width() - 360, 29), 17, GREEN)
        self.draw_world()
        self.draw_side()
        for button in self.buttons:
            color = (41, 89, 145) if button.active else (33, 47, 66)
            pygame.draw.rect(self.screen, color, button.rect, border_radius=7)
            size = 13 if button.rect.height < 30 else 15
            self.text(button.label, (button.rect.x + 12, button.rect.y + (button.rect.height - size) // 2 - 1), size)
        gap, width = 12, (self.graph_area.width - 24) // 3
        plots = [pygame.Rect(self.graph_area.x + i * (width + gap), self.graph_area.y, width, 150) for i in range(3)]
        self.graph(plots[0], 'Filtered Fn (red) / target (gold) / raw (grey)',
                   [(lambda r: r['raw_fx_n']*r['nx']+r['raw_fy_n']*r['ny'], MUTED),
                    (lambda r: r['fn_n'], RED), (lambda r: r['target_n'], GOLD)],
                   0, max(3, self.exp.settings.target_force * 1.8), 'N')
        self.graph(plots[1], 'Tool (gold) / n estimate (cyan) / truth (grey)',
                   [(lambda r: r['tool_deg'], GOLD), (lambda r: r['normal_deg'], CYAN),
                    (lambda r: r['true_normal_deg'], MUTED)], -95, 95, 'deg')
        self.graph(plots[2], 'Tangent v actual (green) / commanded (blue)',
                   [(lambda r: r['actual_tangent_mps'] * 1000, GREEN), (lambda r: r['feed_mps'] * 1000, BLUE)],
                   -2, max(12, self.exp.settings.feed * 1400), 'mm/s')
        y = self.screen.get_height() - 30
        self.text(self.exp.controller.reason or self.message, (22, y), 13,
                  RED if self.exp.controller.phase == 'STOPPED' else MUTED)
        pygame.display.flip()

    def events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode((max(1170, event.w), max(880, event.h)), pygame.RESIZABLE)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                for button in self.buttons:
                    if button.rect.collidepoint(event.pos):
                        button.callback()
                        break
                for slider in self.sliders:
                    if slider.rect.inflate(0, 24).collidepoint(event.pos):
                        self.dragging = slider
                        self.edit_slider(slider, event.pos[0])
            elif event.type == pygame.MOUSEMOTION and self.dragging:
                self.edit_slider(self.dragging, event.pos[0])
            elif event.type == pygame.MOUSEBUTTONUP:
                self.dragging = None
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                if event.key == pygame.K_SPACE:
                    self.toggle_pause()
                elif event.key == pygame.K_r:
                    self.exp.reset()
                elif event.key == pygame.K_h:
                    self.change('controller', 'hybrid')
                elif event.key == pygame.K_a:
                    self.change('controller', 'admittance')
                elif pygame.K_1 <= event.key <= pygame.K_4:
                    self.change('scene', Terrain.names[event.key - pygame.K_1])
                elif event.key == pygame.K_s:
                    self.save()
                elif event.key == pygame.K_p:
                    self.exp.physics.disturb()
                elif event.key in (pygame.K_F1, pygame.K_F2):
                    self.change_preset('ideal' if event.key == pygame.K_F1 else 'robot')
        return True

    def run(self, duration=None, screenshot=None):
        accumulator = 0.0
        self.draw()
        try:
            while self.events():
                wall_dt = min(self.clock.tick(60) / 1000, .08)
                if not self.paused and not self.exp.stopped:
                    accumulator += wall_dt * self.exp.settings.time_scale
                    dt = 1 / self.exp.settings.physics_hz
                    while accumulator >= dt and not self.exp.stopped:
                        self.exp.step()
                        accumulator -= dt
                        if duration is not None and self.exp.time >= duration:
                            break
                else:
                    accumulator = 0
                self.draw()
                if self.exp.paused:
                    self.paused = True
                if duration is not None and (self.exp.time >= duration or self.exp.stopped):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            if screenshot:
                screenshot.parent.mkdir(parents=True, exist_ok=True)
                pygame.image.save(self.screen, str(screenshot))
            pygame.quit()
