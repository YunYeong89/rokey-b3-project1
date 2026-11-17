"""Stock check in the repair DB for the parts to replace (session/scan_result.json `replace`, or --parts).
The result goes to /workspace/session/stock_result.json; with --replace, replace_parts runs for the parts
in stock. Needs the web server (robot.yaml `db.url`, e.g. ./run_virtual.sh --web).
  ros2 run rokey check_db [--parts bonnet mirror_r] [--replace]
"""
import argparse

if __package__:
    from .configs import load, read_session, write_session
    from .repair_db import RepairDB
else:
    from configs import load, read_session, write_session
    from repair_db import RepairDB


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
    options = parser.parse_args()
    parts = options.parts
    if parts is None:
        scan = read_session('scan_result.json')
        if scan is None:
            parser.error('session/scan_result.json 없음: scan_surface를 먼저 실행하거나 --parts를 지정하세요')
        parts = scan['replace']
    available, missing = StockChecker(RepairDB(**load('robot')['db'])).check(parts)
    path = write_session('stock_result.json', {'parts': parts, 'available': available, 'out_of_stock': missing})
    print(f'교체 대상 {parts}: 재고 있음 {available}, 재고 없음 {missing} -> {path}', flush=True)
    if options.replace and available:
        if __package__:
            from .replace_parts import run
        else:
            from replace_parts import run
        run(available)


if __name__ == '__main__':
    main()
