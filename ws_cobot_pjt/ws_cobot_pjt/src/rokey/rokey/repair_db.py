"""REST client of the repair web server (INTERFACE.md section 4), header X-API-Key. The address and key come
from robot.yaml `db` (environment REPAIR_DB_URL / REPAIR_API_KEY override them)."""
import json
import os
import urllib.error
import urllib.request


class RepairDB:
    def __init__(self, url, api_key, timeout=5.0):
        self.url = os.environ.get('REPAIR_DB_URL', url).rstrip('/')
        self.key, self.timeout = os.environ.get('REPAIR_API_KEY', api_key), timeout

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
