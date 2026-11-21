"""REST client of the repair web server (INTERFACE.md section 4), header X-API-Key. The address and key come
from robot.yaml `db` (environment REPAIR_DB_URL / REPAIR_API_KEY override them)."""
import json
import os
import socket
import urllib.error
import urllib.parse
import urllib.request


class RepairDB:
    def __init__(self, url, api_key, timeout=5.0):
        self.url = self.reachable(os.environ.get('REPAIR_DB_URL', url).rstrip('/'))
        self.key, self.timeout = os.environ.get('REPAIR_API_KEY', api_key), timeout

    @staticmethod
    def reachable(url):
        """robot.yaml's host.docker.internal names the host PC from inside the Docker container only; run directly on
        Ubuntu (sodreal + ros2 run) that name does not resolve and the web server is on this PC: localhost."""
        parts = urllib.parse.urlsplit(url)
        if parts.hostname == 'host.docker.internal':
            try:
                socket.getaddrinfo(parts.hostname, None)
            except socket.gaierror:
                return parts._replace(netloc=parts.netloc.replace('host.docker.internal', 'localhost')).geturl()
        return url

    def request(self, method, path, body=None, allow=()):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(self.url + path, data=data, method=method,
                                         headers={'X-API-Key': self.key, 'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read() or 'null')
        except urllib.error.HTTPError as error:
            if error.code in allow:
                return json.loads(error.read() or 'null')
            raise RuntimeError(f'DB {method} {path}: HTTP {error.code} {error.read().decode(errors="replace")}')
        except urllib.error.URLError as error:
            raise RuntimeError(f'DB 서버에 연결할 수 없습니다 ({self.url}): {error.reason}')

    def job(self, job_id):
        return self.request('GET', f'/api/jobs/{job_id}')

    def vehicle(self, vehicle_id):
        for vehicle in self.request('GET', '/api/vehicles'):
            if vehicle['id'] == vehicle_id:
                return vehicle
        raise ValueError('등록된 차량을 선택하세요')

    def requested_parts(self, job_id):
        return self.job(job_id)['requested']

    def event(self, job_id, stage, message, level='info'):
        return self.request('POST', f'/api/jobs/{job_id}/events', {'stage': stage, 'message': message, 'level': level})

    def findings(self, job_id, items):
        return self.request('POST', f'/api/jobs/{job_id}/findings', items)

    def part_available(self, code):
        return bool(self.request('GET', f'/api/parts/{code}/check', allow=(404,)).get('available'))

    def replacement(self, job_id, code, success, retries=0, peak_force=None, flush_mm=None):
        return self.request('POST', f'/api/jobs/{job_id}/replacements',
                            {'part_code': code, 'success': success, 'retries': retries,
                             'peak_force': peak_force, 'flush_mm': flush_mm})

    def consume(self, code):
        return self.request('POST', f'/api/parts/{code}/consume')

    def finish(self, job_id, status, summary=''):
        return self.request('POST', f'/api/jobs/{job_id}/finish', {'status': status, 'summary': summary})
