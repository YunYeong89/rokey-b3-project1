"""python -m turnlift {report|plan|sdf|mjcf|mujoco|table} [--config device.yaml] [--gui] [--serve PORT]

  report  masses, lift servo torque, in-place turn, holding against the probe force, play, sweep vs robot HOME
  plan    probe-able vehicle surface per part over headings x lift states x {top, front} (slow: minutes)
  sdf     Gazebo model + world of the stand with the vehicle -> out/models/turn_lift, out/worlds/turn_lift.sdf
  mjcf    MuJoCo model of the same stand -> out/mujoco/turn_lift.xml (compiled to check)
  mujoco  the unchanged firmware driving the MuJoCo stand: calibration, turns, stations, lift, holding against probe
          forces, believed vs true (--gui: viewer, on macOS run with mjpython; --serve PORT: the firmware TCP protocol
          on the simulated stand instead)
  table   servo angle <-> lift height table for the firmware -> firmware/lift_table.json
Results also go to out/<command>.json."""
import argparse
import json

import numpy as np

from .config import OUT, ROOT, load
from .device import TurnLift, apply
from .loads import Holding, LiftTorque, Play, TurnTorque
from .plan import ScanPlan
from .sdf import SdfExport


def rounded(value):
    if isinstance(value, dict):
        return {k: rounded(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [rounded(v) for v in value]
    return round(float(value), 3) if isinstance(value, (float, np.floating)) else value


def show(title, rows):
    print(f'\n## {title}')
    for row in rows:
        print('  ' + ', '.join(f'{k} {v}' for k, v in rounded(row).items()))


def report(device):
    c, lift = device.c, device.lift
    down, up = c['lift']['servo_down_up_deg']
    result = dict(masses=dict(lift_parts=[{k: p[k] for k in ('part', 'count', 'mass')} for p in lift.parts],
                              lift=lift.mass, lift_rising=lift.rising, adapter=device.adapter_mass,
                              vehicle=device.vehicle.mass, total_down=device.total(down)[0]))
    show('masses kg', [{k: v for k, v in result['masses'].items() if k != 'lift_parts'}])
    result['heights'] = dict(top_down_mm=lift.top(down), top_up_mm=lift.top(up), vehicle_ground_down_mm=device.plate_z(down),
                             vehicle_ground_up_mm=device.plate_z(up), vehicle_top_up_mm=device.plate_z(up) + device.vehicle.size[2])
    show('heights (table z = 0)', [result['heights']])
    torque = LiftTorque(device)
    result['lift'] = [torque.check(f) for f in (0., c['probe']['force_n'], c['probe']['max_force_n'])]
    show('lift servo (MG996R)', result['lift'])
    turn = TurnTorque(device)
    result['turn'] = [turn.check(s) for s in (down, up)]
    show('in-place turn (TT motors)', result['turn'])
    holding, rows = Holding(device), []
    centre = np.array(c['placement']['centre_mm'])
    for heading in c['scan']['headings_deg']:
        for servo in (down, up):
            points = apply(device.vehicle_pose(heading, servo), device.vehicle.triangles.reshape(-1, 3)[::5])
            high = points[:, 2] > points[:, 2].max() - 30
            top = points[high][np.argmax(np.linalg.norm(points[high, :2] - centre, axis=1))]
            front = points[np.argmin(points[:, 0])]
            for name, point, direction in (('top', top, [0, 0, -1.]), ('front', front, [1., 0, 0])):
                for force in (c['probe']['force_n'], c['probe']['max_force_n']):
                    rows.append(dict(load=name, force_n=force, **holding.check(heading, servo, point, force * np.array(direction))))
    result['holding'] = rows
    keys = ('tip_margin_mm', 'slide_x_coast', 'slide_x_brake', 'slide_y', 'yaw')
    worst = [dict(load=n, force_n=f, **{k: min(r[k] for r in rows if r['load'] == n and r['force_n'] == f) for k in keys})
             for n in ('top', 'front') for f in (c['probe']['force_n'], c['probe']['max_force_n'])]
    show('holding, worst over headings and lift (tip margin mm; others = capacity / load)', worst)
    result['play'] = [Play(device).check(s, device.vehicle.size[2]) for s in (down, up)]
    show('top plate play (worst case)', result['play'])
    home = np.array(c['robot']['home_tip_mm'])
    sweep = [dict(servo_deg=s, sweep_mm=device.sweep(s), home_distance_mm=float(np.linalg.norm(home[:2] - centre)),
                  vehicle_top_mm=device.plate_z(s) + device.vehicle.size[2], home_z_mm=home[2]) for s in (down, up)]
    result['sweep'] = sweep
    show('turning sweep vs robot HOME tip', sweep)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('report', 'plan', 'sdf', 'mjcf', 'mujoco', 'table'))
    parser.add_argument('--config')
    parser.add_argument('--gui', action='store_true', help='mujoco: passive viewer (macOS: mjpython -m turnlift ...)')
    parser.add_argument('--serve', type=int, metavar='PORT', help='mujoco: firmware TCP protocol on the simulation')
    options = parser.parse_args()
    device = TurnLift(load(options.config))
    OUT.mkdir(exist_ok=True)
    if options.command == 'report':
        result = report(device)
    elif options.command == 'plan':
        rows, coverage = ScanPlan(device).run()
        result = dict(states=rows, coverage=coverage)
        show('coverage over all states (reachable / visible samples)', [dict(part=p, **v) for p, v in coverage.items()])
    elif options.command == 'sdf':
        folder = SdfExport(device).write(OUT)
        result = dict(model=str(folder))
        print(f'{folder}/model.sdf, {OUT}/worlds/turn_lift.sdf')
    elif options.command == 'mjcf':
        import mujoco
        from .mjcf import MjcfExport
        path = MjcfExport(device).write(OUT / 'mujoco')
        model = mujoco.MjModel.from_xml_path(str(path))
        result = dict(model=str(path), bodies=model.nbody, joints=model.njnt, actuators=model.nu)
        print(f'{path}: {model.nbody} bodies, {model.njnt} joints, {model.nu} actuators')
    elif options.command == 'mujoco':
        from .mujoco_sim import Scenario, TurnLiftSim
        sim = TurnLiftSim(device, options.gui)
        try:
            scenario = Scenario(sim)
            result = scenario.serve(options.serve) if options.serve else scenario.run()
        finally:
            sim.close()
        for key, rows in result.items():
            if isinstance(rows, list):
                show(f'mujoco {key}', rows)
    else:
        lift = device.lift
        result = dict(servo_deg=[round(float(a), 2) for a in lift.angles(141)],
                      top_mm=[round(lift.top(a), 2) for a in lift.angles(141)])
        (ROOT / 'firmware/lift_table.json').write_text(json.dumps(result))
        print(f'{ROOT}/firmware/lift_table.json: {result["top_mm"][0]} -> {result["top_mm"][-1]} mm')
    (OUT / f'{options.command}.json').write_text(json.dumps(rounded(result), indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
