"""Stock check in the repair DB for the parts to replace (data/scan_result.json `replace`, or --parts).
The result goes to /workspace/src/repair_car/data/stock_result.json; with --replace, replace_parts runs for the parts
in stock. Needs the web server (robot.yaml `db.url`, e.g. ./run_virtual.sh --web).
  ros2 run repair_car check_db [--parts bonnet mirror_r] [--replace]
"""
import argparse

if __package__:
    from repair_car.robot_control.robot_bridge import program_args
else:
    from robot_control.robot_bridge import program_args

if __package__:
    from repair_car.config import load, read_session, write_session
    from repair_car.web.repair_db import RepairDB
else:
    from config import load, read_session, write_session
    from repair_car.web.repair_db import RepairDB


class StockChecker:
    def __init__(self, db):
        self.db = db

    def check(self, parts):
        """(in stock, out of stock) part codes."""
        available = {part: self.db.part_available(part) for part in parts}
        return [part for part, ok in available.items() if ok], [part for part, ok in available.items() if not ok]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--parts', nargs='+', help='part codes (default: scan_result.json replace)')
    parser.add_argument('--replace', action='store_true', help='run replace_parts for the parts in stock')
    options = parser.parse_args(program_args())
    parts = options.parts
    if parts is None:
        scan = read_session('scan_result.json')
        if scan is None:
            parser.error('data/scan_result.json missing: run scan_surface first or give --parts')
        parts = scan['replace']
    available, missing = StockChecker(RepairDB(**load('robot')['db'])).check(parts)
    path = write_session('stock_result.json', {'parts': parts, 'available': available, 'out_of_stock': missing})
    print(f'Parts to replace {parts}: in stock {available}, out of stock {missing} -> {path}', flush=True)
    if options.replace and available:
        if __package__:
            from repair_car.part_replace.replace_parts import run
        else:
            from part_replace.replace_parts import run
        run(available)


if __name__ == '__main__':
    main()
