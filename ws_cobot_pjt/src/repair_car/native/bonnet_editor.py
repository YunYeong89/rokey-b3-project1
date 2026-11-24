"""Open the local bonnet inspection-area editor. Saving writes coordinates only."""
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
import webbrowser


ROOT = Path(__file__).resolve().parents[4]
HTML = Path(__file__).with_name('bonnet_path.html')
CAD = ROOT / 'ws_cobot_pjt/src/repair_car/native/.cache/scene/vehicle/real_bonnet.stl'
SELECTION = ROOT / 'ws_cobot_pjt/src/repair_car/config/bonnet_path.json'


class BonnetEditor(BaseHTTPRequestHandler):
    def respond(self, data, content_type):
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/':
            self.respond(HTML.read_bytes(), 'text/html; charset=utf-8')
        elif self.path == '/bonnet.stl' and CAD.is_file():
            self.respond(CAD.read_bytes(), 'application/octet-stream')
        elif self.path == '/selection':
            data = SELECTION.read_bytes() if SELECTION.is_file() else b'{"regions": []}'
            self.respond(data, 'application/json')
        else:
            self.send_error(404)

    def do_PUT(self):
        size = int(self.headers.get('Content-Length', '0'))
        if self.path != '/selection' or self.headers.get_content_type() != 'application/json' or not 0 < size < 100000:
            self.send_error(400)
            return
        try:
            data = json.loads(self.rfile.read(size))
            from repair_car.inspection.part_scan.bonnet_path import BonnetPath

            BonnetPath(data['regions'])
        except (ValueError, KeyError, TypeError) as error:
            self.respond(json.dumps({'error': str(error)}).encode(), 'application/json')
            return
        SELECTION.parent.mkdir(parents=True, exist_ok=True)
        temporary = SELECTION.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, indent=2) + '\n')
        temporary.replace(SELECTION)
        self.respond(b'{}', 'application/json')


def main():
    import sys

    sys.path.insert(0, str(ROOT / 'ws_cobot_pjt/src/repair_car'))
    if not CAD.is_file():
        from repair_car.geometry.real_vehicle import RealVehicle

        RealVehicle().export(CAD.parent)
    with HTTPServer(('127.0.0.1', 0), BonnetEditor) as server:
        url = f'http://127.0.0.1:{server.server_port}/'
        print(url, flush=True)
        webbrowser.open(url)
        server.serve_forever()


if __name__ == '__main__':
    main()
