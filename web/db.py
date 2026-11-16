"""SQLite 연결과 초기화

repair.db 가 새로 만들어질 때(users 가 비어 있을 때) 데이터를 넣는다.
  - 기본: 계정 admin / user1 / user2, 부품 전체, 차량 모형 A-01, A-02
  - 샘플 (기본값): 위에 더해 회원 36명 · 차량 40대 · 지난 30일 수리 이력 100건 (발표·테스트용)
  - 환경변수 REPAIR_SAMPLE=0 이면 샘플 없이 기본 데이터만 (실제 로봇 시연용)
처음 상태로 되돌리려면 repair.db 를 지우고 다시 실행.
"""
import os
import random
import sqlite3
from datetime import datetime, timedelta

from flask import g
from werkzeug.security import generate_password_hash

BASE = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.environ.get("REPAIR_DB", os.path.join(BASE, "repair.db"))
FMT = "%Y-%m-%d %H:%M:%S"
SAMPLE_SEED = 42

# 부품 목록: (code, 이름, 분류, robot_ok 기본값, 창고 위치). code 는 로봇·화면과 약속된 값이라 바꾸지 말 것
PARTS = [
    ("bonnet", "보닛", "외판", 1, "A-01"),
    ("door_fl", "앞문(좌)", "외판", 1, "A-02"),
    ("door_fr", "앞문(우)", "외판", 1, "A-03"),
    ("door_rl", "뒷문(좌)", "외판", 1, "A-04"),
    ("door_rr", "뒷문(우)", "외판", 1, "A-05"),
    ("bumper_f", "앞범퍼", "외판", 0, "A-06"),
    ("bumper_r", "뒷범퍼", "외판", 0, "A-07"),
    ("trunk", "트렁크 리드", "외판", 0, "A-08"),
    ("fender_l", "펜더(좌)", "외판", 0, "A-09"),
    ("fender_r", "펜더(우)", "외판", 0, "A-10"),
    ("mirror_l", "사이드미러(좌)", "미러", 1, "B-01"),
    ("mirror_r", "사이드미러(우)", "미러", 1, "B-02"),
    ("headlamp_l", "헤드램프(좌)", "램프", 0, "C-01"),
    ("headlamp_r", "헤드램프(우)", "램프", 0, "C-02"),
    ("taillamp_l", "테일램프(좌)", "램프", 0, "C-03"),
    ("taillamp_r", "테일램프(우)", "램프", 0, "C-04"),
    ("wiper", "와이퍼 블레이드", "기타", 0, "D-01"),
]
PART_NAME = {p[0]: p[1] for p in PARTS}


def get_db():
    if "db" not in g:
        g.db = sqlite3.connect(DB_PATH)
        g.db.row_factory = sqlite3.Row
        g.db.execute("PRAGMA foreign_keys = ON")
    return g.db


def close_db(_=None):
    db = g.pop("db", None)
    if db is not None:
        db.close()


def init_db(seed=True):
    con = sqlite3.connect(DB_PATH)
    con.execute("PRAGMA foreign_keys = ON")
    with open(os.path.join(BASE, "schema.sql"), encoding="utf-8") as f:
        con.executescript(f.read())
    if seed and con.execute("SELECT COUNT(*) FROM users").fetchone()[0] == 0:
        with con:
            insert_base(con)
            if os.environ.get("REPAIR_SAMPLE", "1") != "0":
                insert_sample(con)
    con.close()


def rows(cur):
    return [dict(r) for r in cur.fetchall()]


# ================================================================ 기본 데이터
def insert_base(con):
    """기본 계정 (처음 실행 후 비밀번호 바꾸기), 부품 전체, 차량 2대"""
    users = [("admin", "admin1234", "정비소 관리자", "admin"),
             ("user1", "user1234", "차주 1", "user"),
             ("user2", "user1234", "차주 2", "user")]
    for u, pw, name, role in users:
        con.execute("INSERT INTO users (username, pw_hash, name, role) VALUES (?, ?, ?, ?)",
                    (u, generate_password_hash(pw), name, role))
    con.executemany("INSERT OR IGNORE INTO parts (code, name, category, robot_ok, location, stock, min_stock) "
                    "VALUES (?, ?, ?, ?, ?, ?, 1)",
                    [(c, n, cat, ok, loc, 3 if c == "bonnet" else 2) for c, n, cat, ok, loc in PARTS])
    uid = {r[0]: r[1] for r in con.execute("SELECT username, id FROM users")}
    con.executemany("INSERT OR IGNORE INTO vehicles (name, model, owner_id) VALUES (?, ?, ?)", [
        ("모형 A-01", "sedan_1_18", uid["user1"]),
        ("모형 A-02", "sedan_1_18", uid["user2"]),
    ])


# ================================================================ 샘플 데이터 (발표·테스트용)
# 재고: (stock, min_stock). 0 = 없음, min_stock 이하 = 부족
SAMPLE_STOCK = {
    "bonnet": (5, 1), "door_fl": (3, 1), "door_fr": (4, 1), "door_rl": (2, 1), "door_rr": (1, 1),
    "bumper_f": (2, 1), "bumper_r": (3, 1), "trunk": (2, 1), "fender_l": (4, 1), "fender_r": (3, 1),
    "mirror_l": (1, 1), "mirror_r": (0, 1),
    "headlamp_l": (2, 1), "headlamp_r": (3, 1), "taillamp_l": (2, 1), "taillamp_r": (0, 1),
    "wiper": (1, 2),
}
SURNAMES = list("김이박최정강조윤장임한오서신권황안송류홍")
GIVEN = ["민준", "서연", "도윤", "지우", "하준", "서윤", "시우", "하은", "지호", "수아", "예준", "지아",
         "주원", "다은", "건우", "채원", "현우", "소율", "우진", "지민", "선우", "예린", "연우", "윤서",
         "정우", "가은", "승현", "나연", "태윤", "유진", "준혁", "수빈"]
N_JOBS, DAYS, N_MANUAL = 100, 30, 30
STATUS_COUNT = {"done": 62, "no_damage": 12, "on_hold": 10, "failed": 9, "stopped": 7}

# 손상 좌표 범위 [mm] (차량 좌표계, viewer3d.js 임시 모델과 같은 치수)
BONNET_X, BONNET_Y = (180.0, 245.0), (20.0, 90.0)          # 보닛: 위에서 찍음, z 없음
DOOR_X = {"door_rl": (77.0, 113.0), "door_fl": (125.0, 160.0), "door_rr": (77.0, 113.0), "door_fr": (125.0, 160.0)}
DOOR_Y = {"door_rl": 0.0, "door_fl": 0.0, "door_rr": 110.0, "door_fr": 110.0}   # 왼쪽 옆면 y=0, 오른쪽 y=110
DOOR_Z = (16.0, 35.0)
DOORS = list(DOOR_X)
MIRRORS = ["mirror_l", "mirror_r"]


def _fmt(t):
    return t.strftime(FMT)


def insert_sample(con, seed=SAMPLE_SEED):
    """insert_base 다음에 호출. 같은 날짜·같은 seed 이면 같은 데이터"""
    rng = random.Random(seed)
    today = datetime.now().replace(hour=0, minute=0, second=0, microsecond=0)
    uid = _sample_users(con, rng)
    for code, (stock, min_stock) in SAMPLE_STOCK.items():
        con.execute("UPDATE parts SET stock = ?, min_stock = ? WHERE code = ?", (stock, min_stock, code))
    vehicles = _sample_vehicles(con, rng, uid, today)
    _sample_jobs(con, rng, uid, vehicles, today)


def _sample_users(con, rng):
    # 해시는 비밀번호별로 한 번만 만든다 (scrypt 가 느려서 36번 만들면 첫 실행이 몇 초 걸림)
    pw = {p: generate_password_hash(p) for p in ("admin1234", "user1234")}
    names = rng.sample([s + n for s in SURNAMES for n in GIVEN], 32)
    users = [("admin2", "admin1234", "정비소 관리자 2", "admin")]
    users += [(f"user{i}", "user1234", names[i - 3], "user") for i in range(3, 35)]
    con.executemany("INSERT INTO users (username, pw_hash, name, role) VALUES (?, ?, ?, ?)",
                    [(u, pw[p], n, r) for u, p, n, r in users])
    return {r[0]: r[1] for r in con.execute("SELECT username, id FROM users")}


def _sample_vehicles(con, rng, uid, today):
    """A-03 ~ A-40 (38대): 차주 없는 3대 + user3 과 2명은 2대씩, 나머지 1대씩"""
    others = [f"user{i}" for i in range(3, 35)]
    owners = others + ["user3"] + rng.sample(others[1:], 2) + [None] * 3
    rng.shuffle(owners)
    con.executemany("INSERT INTO vehicles (name, model, owner_id, created_at) VALUES (?, ?, ?, ?)", [
        (f"모형 A-{i:02d}", "sedan_1_18", uid[o] if o else None,
         _fmt(today - timedelta(days=60 - i // 2, hours=rng.randint(0, 8))))
        for i, o in enumerate(owners, 3)])
    con.execute("UPDATE vehicles SET created_at = ? WHERE name IN ('모형 A-01', '모형 A-02')",
                (_fmt(today - timedelta(days=61)),))
    return [tuple(r) for r in con.execute("SELECT id, owner_id FROM vehicles ORDER BY id")]


def _sample_jobs(con, rng, uid, vehicles, today):
    statuses = [s for s, n in STATUS_COUNT.items() for _ in range(n)]
    rng.shuffle(statuses)
    manual = set(rng.sample([i for i, s in enumerate(statuses) if s != "no_damage"], N_MANUAL))
    u3 = [vid for vid, o in vehicles if o == uid["user3"]]                 # user3 차량은 이력 2건 이상
    vids = u3 * 2 + [rng.choice(vehicles)[0] for _ in range(N_JOBS - 2 * len(u3))]
    rng.shuffle(vids)
    admins = [uid["admin"], uid["admin2"]]

    for i, start in enumerate(_schedule(rng, today)):
        mode = "manual" if i in manual else "auto"
        j = _build_job(rng, mode, statuses[i])
        end = start + timedelta(minutes=_duration(rng, j))
        jid = con.execute(
            "INSERT INTO jobs (vehicle_id, operator_id, mode, status, summary, started_at, finished_at) "
            "VALUES (?, ?, ?, ?, ?, ?, ?)",
            (vids[i], rng.choices(admins, weights=[6, 4])[0], mode, j.status, j.summary,
             _fmt(start), _fmt(end))).lastrowid
        # 로그 시각: 단계 가중치에 비례해 시작~종료 사이에 배치
        total, span, acc = sum(s[0] for s in j.steps) + 0.5, (end - start).total_seconds(), 0.0
        for w, stage, level, msg, rep in j.steps:
            acc += w
            ts = _fmt(start + timedelta(seconds=span * acc / total))
            con.execute("INSERT INTO events (job_id, ts, stage, level, message) VALUES (?, ?, ?, ?, ?)",
                        (jid, ts, stage, level, msg))
            if rep:
                con.execute("INSERT INTO replacements (job_id, part_code, success, retries, peak_force, flush_mm, ts) "
                            "VALUES (?, ?, ?, ?, ?, ?, ?)", (jid, *rep, ts))
        con.executemany("INSERT INTO findings (job_id, part_code, x, y, z, depth, decision) VALUES (?, ?, ?, ?, ?, ?, ?)",
                        [(jid, *f) for f in j.findings])
        con.execute("INSERT INTO events (job_id, ts, stage, level, message) VALUES (?, ?, 'finish', ?, ?)",
                    (jid, _fmt(end), "info" if j.status in ("done", "no_damage") else "warn", j.summary))


def _schedule(rng, today):
    """지난 30일 9~18시, 작업끼리 겹치지 않게 (간격 16분 이상, 작업은 15분 이하)"""
    per_day = {}
    for d in rng.choices(range(1, DAYS + 1), k=N_JOBS):
        per_day[d] = per_day.get(d, 0) + 1
    slots = []
    for d in sorted(per_day, reverse=True):                               # 오래된 날부터 → id 가 시간 순
        n = per_day[d]
        t = (today - timedelta(days=d)).replace(hour=9) + timedelta(minutes=rng.randint(0, 40))
        for _ in range(n):
            slots.append(t)
            t += timedelta(minutes=rng.uniform(16, max(17, 440 / n)))
    return slots


class _Job:
    def __init__(self):
        self.steps = []          # (가중치, stage, level, message, replacement 행 또는 None)
        self.findings = []       # (part_code, x, y, z, depth, decision)
        self.status = self.summary = None

    def ev(self, stage, message, level="info", weight=1.0, rep=None):
        self.steps.append((weight, stage, level, message, rep))


def _targets(rng, status):
    """교체 대상: 로봇 교체 가능(robot_ok=1) 부품만 — 보닛, 미러, 문"""
    if status == "no_damage":
        return []
    if status == "on_hold":                                               # 재고 0 인 사이드미러(우)가 꼭 포함
        return (["bonnet"] if rng.random() < 0.3 else []) + ["mirror_r"]
    r = rng.random()
    if r < 0.40:
        return ["bonnet"]
    if r < 0.60:
        return [rng.choice(MIRRORS)]
    if r < 0.82:
        return [rng.choice(DOORS)]
    return rng.choice([["bonnet", rng.choice(DOORS)], ["bonnet", rng.choice(MIRRORS)], [rng.choice(DOORS), rng.choice(MIRRORS)]])


def _build_job(rng, mode, status):
    j, name = _Job(), PART_NAME
    targets = _targets(rng, status)
    stop_at = rng.choice(["calibrate", "scan", "replace"] if mode == "auto" else ["calibrate", "replace"]) \
        if status == "stopped" else None

    def stopped():
        j.status, j.summary = "stopped", "관리자가 정지 버튼을 누름"
        return j

    j.ev("start", "수리 시작 (자동 모드)" if mode == "auto" else f"수리 시작 (수동 모드: {', '.join(targets)})", weight=0)
    if mode == "manual":
        j.findings += [(c, None, None, None, None, "requested") for c in targets]
    j.ev("calibrate", "차량 기준점 3곳 터치", weight=2)
    if stop_at == "calibrate":
        return stopped()
    j.ev("calibrate", "차량 좌표계 설정 완료", weight=0.5)

    if mode == "auto":
        dents = []
        if "bonnet" in targets:
            dents += [("bonnet", round(rng.uniform(*BONNET_X), 1), round(rng.uniform(*BONNET_Y), 1), None)
                      for _ in range(rng.choice([1, 1, 1, 2]))]
        for d in (c for c in targets if c in DOOR_X):
            dents.append((d, round(rng.uniform(*DOOR_X[d]), 1), DOOR_Y[d], round(rng.uniform(*DOOR_Z), 1)))
        j.ev("scan", "표면 스캔(연속 접촉) 시작 — 윗면 3구역, 라인 간격 15 mm, 점 간격 5 mm", weight=1 if dents else 4)
        if any(d[0] == "bonnet" for d in dents):
            j.ev("scan", f"의심 구역 {rng.randint(2, 5)}곳 상세 스캔(연속 접촉, 라인 간격 4 mm)", weight=3)
        j.ev("scan", "표면 스캔(연속 접촉) — 옆면 문 4곳", weight=2)
        for d in dict.fromkeys(c for c in targets if c in DOOR_X):
            j.ev("scan", f"{name[d]} 의심 구역 상세 스캔(연속 접촉)", "warn", 1)
        if stop_at == "scan":
            return stopped()
        for m in MIRRORS:
            miss = m in targets
            j.ev("scan", f"{name[m]} 접촉 확인: {'접촉 없음' if miss else '접촉 있음'}", "warn" if miss else "info", 0.5)
        j.ev("detect", "정상 모델과 비교 중", weight=1)
        j.findings += [(c, x, y, z, -round(rng.uniform(0.3, 3.0), 2), "dent") for c, x, y, z in dents]
        j.findings += [(m, None, None, None, None, "missing") for m in targets if m in MIRRORS]
        if not targets:
            j.ev("detect", "손상 없음", weight=0.2)
            j.status, j.summary = "no_damage", "모든 측정 점이 임계값 이내"
            return j
        j.ev("detect", f"교체 대상: {', '.join(name[c] for c in targets)}", "warn", 0.2)
    else:
        j.ev("stock", f"수동 모드: 지정 부품 {', '.join(name[c] for c in targets)} 교체 (스캔·판정 건너뜀)", weight=0.2)

    lacking = []                                                          # 과거 재고는 지금과 달랐다고 보고, 보류 작업만 재고 0
    for c in targets:
        if status == "on_hold" and c == "mirror_r":
            j.ev("stock", f"{name[c]} 재고 0개", "warn", 0.3)
            lacking.append(name[c])
        else:
            j.ev("stock", f"{name[c]} 재고 {rng.randint(1, 6)}개", weight=0.3)
    if lacking:
        j.status, j.summary = "on_hold", f"재고 없음: {', '.join(lacking)}"
        return j

    fail_kind = rng.choice(["flush", "flush", "flush", "fault"]) if status == "failed" else None
    fail_idx = rng.randrange(len(targets)) if status == "failed" else -1
    results = []
    for i, c in enumerate(targets):
        j.ev("replace", f"{name[c]} 분리", weight=3)
        if stop_at == "replace":
            return stopped()
        retries = 0
        if rng.random() < 0.12:
            retries = 1
            j.ev("replace", f"{name[c]} 장착 위치 오차 {rng.uniform(0.8, 1.6):.1f} mm, 재정렬 후 재시도", "warn", 1)
        j.ev("replace", f"{name[c]} 새 부품 장착 (힘 제어)", weight=4)
        if i == fail_idx and fail_kind == "fault":
            msg = rng.choice(["협동로봇 보호 정지 (충돌 감지)", "그리퍼 파지 실패 (부품 미끄러짐)", "힘 제어 한계 초과, 장착 중단"])
            j.ev("replace", f"로봇 오류: {msg}", "error", 0.5)
            j.status, j.summary = "failed", msg
            return j
        ok = i != fail_idx
        flush = round(rng.uniform(0.05, 0.45) if ok else rng.uniform(0.5, 0.95), 2)
        j.ev("inspect", f"{name[c]} 단차 측정 (4점)", weight=1.5)
        j.ev("inspect", f"{name[c]} 단차 {flush:.2f} mm → {'합격' if ok else '불합격'}", "info" if ok else "error", 0.2,
             (c, int(ok), retries, round(rng.uniform(10, 25), 1), flush))
        results.append(ok)
    j.ev("done", "작업 종료, 홈 위치 복귀", weight=1.5)
    if all(results):
        j.status, j.summary = "done", f"{', '.join(name[c] for c in targets)} 교체 완료"
    else:
        j.status, j.summary = "failed", "재검사 불합격 부품 있음"
    return j


def _duration(rng, j):
    """시작~종료 [분]: 교체 부품 수에 따라 3~15분"""
    n_rep = sum(1 for s in j.steps if s[4])
    if j.status in ("no_damage", "on_hold", "stopped") or (j.status == "failed" and n_rep == 0):
        return rng.uniform(3, 8)
    return rng.uniform(*{0: (3, 7), 1: (6, 11)}.get(n_rep, (10, 15)))
