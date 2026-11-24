"""SQLite connection and initialization

Fills in data when repair.db is newly created (users is empty).
  - Default: accounts admin / user1 / user2, all parts, 2 Land Rover vehicles (real robot and Gazebo)
  - REPAIR_SAMPLE=1: adds 36 members · 40 vehicles · 100 repair records over the last 30 days
  - Environment variable REPAIR_SAMPLE=0: default data only, no samples (for real robot demos)
To reset, delete repair.db and run again.
"""
import json
import os
import random
import sqlite3
import struct
from pathlib import Path
from datetime import datetime, timedelta

from flask import g
from werkzeug.security import generate_password_hash

BASE = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.environ.get("REPAIR_DB", str(Path(BASE).parents[1] / "data/web/repair.db"))
FMT = "%Y-%m-%d %H:%M:%S"
SAMPLE_SEED = 42

# parts list: (code, name, category, default robot_ok, storage location). code is a value agreed with the robot and the pages, do not change it
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


# old repair.db files (schema without part category, robot replacement, location, findings.z) get the columns added instead of being deleted
NEW_COLUMNS = {"parts": [("category", "TEXT"), ("robot_ok", "INTEGER NOT NULL DEFAULT 0 CHECK (robot_ok IN (0, 1))"),
                         ("location", "TEXT")],
               "findings": [("z", "REAL")]}


def migrate_parts(con):
    """Add missing columns + fill the parts list (PARTS). Existing parts keep their stock and job records."""
    added = False
    for table, columns in NEW_COLUMNS.items():
        have = {row[1] for row in con.execute(f"PRAGMA table_info({table})")}
        for name, kind in columns:
            if name not in have:
                con.execute(f"ALTER TABLE {table} ADD COLUMN {name} {kind}")
                added = True
    if added and con.execute("SELECT COUNT(*) FROM users").fetchone()[0]:
        con.executemany("INSERT OR IGNORE INTO parts (code, name, category, robot_ok, location, stock, min_stock) "
                        "VALUES (?, ?, ?, ?, ?, 2, 1)", PARTS)
        con.executemany("UPDATE parts SET category = ?, robot_ok = ?, location = ? WHERE code = ? AND category IS NULL",
                        [(cat, ok, loc, c) for c, _, cat, ok, loc in PARTS])


def init_db(seed=True):
    Path(DB_PATH).parent.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(DB_PATH)
    con.execute("PRAGMA foreign_keys = ON")
    with open(os.path.join(BASE, "schema.sql"), encoding="utf-8") as f:
        con.executescript(f.read())
    migrate(con)
    with con:
        migrate_parts(con)
    if seed and con.execute("SELECT COUNT(*) FROM users").fetchone()[0] == 0:
        with con:
            insert_base(con)
            if os.environ.get("REPAIR_SAMPLE", "0") != "0":
                insert_sample(con)
    con.close()


def migrate(con):
    """Add new columns to an old repair.db (safe to run repeatedly)"""
    with con:
        if "robot_namespace" not in {r[1] for r in con.execute("PRAGMA table_info(vehicles)")}:
            con.execute("ALTER TABLE vehicles ADD COLUMN robot_namespace TEXT")
        for (name, model), (old, robot) in zip(BASE_VEHICLES, BASE_ROBOTS):
            con.execute("""UPDATE vehicles SET name = ?, model = ?, robot_namespace = ?
                           WHERE name = ? AND NOT EXISTS (SELECT 1 FROM vehicles WHERE name = ?)""",
                        (name, model, robot, old, name))
            con.execute("UPDATE vehicles SET robot_namespace = ? WHERE name = ? AND robot_namespace IS NULL",
                        (robot, name))
    cols = {r[1] for r in con.execute("PRAGMA table_info(users)")}
    if "email" not in cols:
        with con:
            con.execute("ALTER TABLE users ADD COLUMN email TEXT")
    old = con.execute("SELECT id, name FROM vehicles WHERE name LIKE '모형 A-%' ORDER BY id").fetchall()
    if old:     # old vehicle names '모형 A-xx' → car model + plate (A-01, A-02 are the base vehicles, the rest random, fixed by id)
        used = {r[0].split(" ", 1)[1] for r in con.execute("SELECT name FROM vehicles WHERE name NOT LIKE '모형 A-%'")
                if " " in r[0]} | {name.split(" ", 1)[1] for name, _ in BASE_VEHICLES}
        base = dict(zip(("모형 A-01", "모형 A-02"), BASE_VEHICLES))
        with con:
            for vid, name in old:
                new = base.get(name) or vehicle_name(random.Random(vid), used)
                con.execute("UPDATE vehicles SET name = ?, model = ? WHERE id = ?", (*new, vid))
    with con:   # old default account names '차주 1/2' → personal names (names changed by an admin are kept)
        con.executemany("UPDATE users SET name = ? WHERE username = ? AND name = ?",
                        [("김태호", "user1", "차주 1"), ("박지영", "user2", "차주 2")])
        for (name, _), (_, robot) in zip(BASE_VEHICLES, BASE_ROBOTS):
            con.execute("UPDATE vehicles SET robot_namespace = ? WHERE name = ? AND robot_namespace IS NULL",
                        (robot, name))


def rows(cur):
    return [dict(r) for r in cur.fetchall()]


# ================================================================ default data
def insert_base(con):
    """Default accounts (change the passwords after the first run), all parts, 2 vehicles"""
    users = [("admin", "admin1234", "정비소 관리자", "admin"),
             ("user1", "user1234", "김태호", "user"),
             ("user2", "user1234", "박지영", "user")]
    for u, pw, name, role in users:
        con.execute("INSERT INTO users (username, pw_hash, name, role) VALUES (?, ?, ?, ?)",
                    (u, generate_password_hash(pw), name, role))
    con.executemany("INSERT OR IGNORE INTO parts (code, name, category, robot_ok, location, stock, min_stock) "
                    "VALUES (?, ?, ?, ?, ?, ?, 1)",
                    [(c, n, cat, ok, loc, 3 if c == "bonnet" else 2) for c, n, cat, ok, loc in PARTS])
    uid = {r[0]: r[1] for r in con.execute("SELECT username, id FROM users")}
    con.executemany("INSERT OR IGNORE INTO vehicles (name, model, owner_id, robot_namespace) VALUES (?, ?, ?, ?)", [
        (*vehicle, uid[user], robot)
        for vehicle, (_, robot), user in zip(BASE_VEHICLES, BASE_ROBOTS, ("user1", "user2"))
    ])


# ================================================================ sample data (presentations and tests)
# stock: (stock, min_stock). 0 = none, at or below min_stock = low
SAMPLE_STOCK = {
    "bonnet": (5, 1), "door_fl": (3, 1), "door_fr": (4, 1), "door_rl": (2, 1), "door_rr": (1, 1),
    "bumper_f": (2, 1), "bumper_r": (3, 1), "trunk": (2, 1), "fender_l": (4, 1), "fender_r": (3, 1),
    "mirror_l": (1, 1), "mirror_r": (0, 1),
    "headlamp_l": (2, 1), "headlamp_r": (3, 1), "taillamp_l": (2, 1), "taillamp_r": (0, 1),
    "wiper": (1, 2),
}
# vehicle name = car model + plate (also the Plate of real_scan reports). The model code prefix is the 3D model kind (viewer3d vehicleType)
CAR_TYPES = [("쏘나타", "sedan"), ("아반떼", "sedan"), ("그랜저", "sedan"), ("K5", "sedan"), ("K8", "sedan"),
             ("G80", "sedan"), ("투싼", "suv"), ("싼타페", "suv"), ("쏘렌토", "suv"), ("스포티지", "suv"),
             ("팰리세이드", "suv"), ("셀토스", "suv"), ("스팅어", "sports"), ("아반떼 N", "sports")]
PLATE_LETTERS = "가나다라마거너더러머버서어저고노도로모보소오조구누두루무부수우주"
BASE_VEHICLES = [("랜드로버 12가 3456", "suv_1_18"), ("랜드로버 34나 5678", "suv_1_18")]
BASE_ROBOTS = [("쏘나타 12가 3456", "robot_1"), ("투싼 34나 5678", "robot_2")]


def vehicle_name(rng, used):
    """(name, model code): car model + a unique plate (e.g. 그랜저 157더 2481)"""
    while True:
        car, kind = rng.choice(CAR_TYPES)
        plate = f"{rng.randint(10, 399)}{rng.choice(PLATE_LETTERS)} {rng.randint(1000, 9999)}"
        if plate not in used:
            used.add(plate)
            return f"{car} {plate}", f"{kind}_1_18"


SURNAMES = list("김이박최정강조윤장임한오서신권황안송류홍")
GIVEN = ["민준", "서연", "도윤", "지우", "하준", "서윤", "시우", "하은", "지호", "수아", "예준", "지아",
         "주원", "다은", "건우", "채원", "현우", "소율", "우진", "지민", "선우", "예린", "연우", "윤서",
         "정우", "가은", "승현", "나연", "태윤", "유진", "준혁", "수빈"]
N_JOBS, DAYS, N_MANUAL = 100, 30, 30
STATUS_COUNT = {"done": 62, "no_damage": 12, "on_hold": 10, "failed": 9, "stopped": 7}

# damage coordinate ranges [mm] (vehicle frame: x front, y right, z height)
# defaults = the viewer3d.js placeholder model size. Replaced by the layout in static/models/car.glb (extras.car_layout written by car_to_glb.py) when present
BONNET_X, BONNET_Y = (180.0, 245.0), (20.0, 90.0)          # bonnet: probed from above, no z
DOOR_X = {"door_rl": (77.0, 113.0), "door_fl": (125.0, 160.0), "door_rr": (77.0, 113.0), "door_fr": (125.0, 160.0)}
DOOR_Y = {"door_rl": 0.0, "door_fl": 0.0, "door_rr": 110.0, "door_fr": 110.0}   # left side y=0, right side y=110
DOOR_Z = (16.0, 35.0)
N_TOP_AREAS = 3                                             # number of top scan areas (for the event text)
CAR_GLB = os.path.join(BASE, "static", "models", "car.glb")


def read_car_layout(path=CAR_GLB):
    """scene extras.car_layout of car.glb (standard library only, no three.js). None when missing or unreadable"""
    try:
        with open(path, "rb") as f:
            magic, _, _, jlen, jtype = struct.unpack("<IIIII", f.read(20))
            if magic != 0x46546C67 or jtype != 0x4E4F534A:
                return None
            j = json.loads(f.read(jlen))
        return j["scenes"][j.get("scene", 0)].get("extras", {}).get("car_layout")
    except (OSError, ValueError, KeyError, IndexError, struct.error):
        return None


def _apply_layout(lay):
    """Change the damage coordinate ranges to the real model layout (same inner margin as admin.js computeScanGeometry on the page)"""
    global BONNET_X, BONNET_Y, DOOR_X, DOOR_Y, DOOR_Z, N_TOP_AREAS
    ins = lambda r, m: (round(r[0] + m, 1), round(r[1] - m, 1))
    b = lay.get("parts", {}).get("bonnet")
    if b:   # middle 60% of the bonnet (avoids the curved edges)
        BONNET_X = ins(b["x"], (b["x"][1] - b["x"][0]) * 0.2)
        BONNET_Y = ins(b["y"], (b["y"][1] - b["y"][0]) * 0.2)
    doors = lay.get("doors") or {}
    if all(d in doors for d in DOOR_X):
        DOOR_X = {k: ins(doors[k]["x"], 5) for k in DOOR_X}
        DOOR_Y = {k: float(doors[k]["y"]) for k in DOOR_X}
        DOOR_Z = ins(doors["door_fl"]["z"], 3)
    N_TOP_AREAS = sum(1 for k in ("trunk", "roof", "bonnet") if k in lay.get("parts", {})) or N_TOP_AREAS


CAR_LAYOUT = read_car_layout()
if CAR_LAYOUT:
    _apply_layout(CAR_LAYOUT)
DOORS = list(DOOR_X)
MIRRORS = ["mirror_l", "mirror_r"]


def _fmt(t):
    return t.strftime(FMT)


def insert_sample(con, seed=SAMPLE_SEED):
    """Called after insert_base. The same date and seed give the same data"""
    rng = random.Random(seed)
    today = datetime.now().replace(hour=0, minute=0, second=0, microsecond=0)
    uid = _sample_users(con, rng)
    for code, (stock, min_stock) in SAMPLE_STOCK.items():
        con.execute("UPDATE parts SET stock = ?, min_stock = ? WHERE code = ?", (stock, min_stock, code))
    vehicles = _sample_vehicles(con, rng, uid, today)
    _sample_jobs(con, rng, uid, vehicles, today)


def _sample_users(con, rng):
    # one hash per password (scrypt is slow: 36 hashes would make the first run take seconds)
    pw = {p: generate_password_hash(p) for p in ("admin1234", "user1234")}
    names = rng.sample([s + n for s in SURNAMES for n in GIVEN], 32)
    users = [("admin2", "admin1234", "정비소 관리자 2", "admin")]
    users += [(f"user{i}", "user1234", names[i - 3], "user") for i in range(3, 35)]
    con.executemany("INSERT INTO users (username, pw_hash, name, role) VALUES (?, ?, ?, ?)",
                    [(u, pw[p], n, r) for u, p, n, r in users])
    return {r[0]: r[1] for r in con.execute("SELECT username, id FROM users")}


def _sample_vehicles(con, rng, uid, today):
    """38 sample vehicles (real car models + plates): 3 without owners + 2 each for user3 and 2 others, 1 each for the rest"""
    others = [f"user{i}" for i in range(3, 35)]
    owners = others + ["user3"] + rng.sample(others[1:], 2) + [None] * 3
    rng.shuffle(owners)
    used = {name.split(" ", 1)[1] for name, _ in BASE_VEHICLES}
    con.executemany("INSERT INTO vehicles (name, model, owner_id, created_at) VALUES (?, ?, ?, ?)", [
        (*vehicle_name(rng, used), uid[o] if o else None,
         _fmt(today - timedelta(days=60 - i // 2, hours=rng.randint(0, 8))))
        for i, o in enumerate(owners, 3)])
    con.execute("UPDATE vehicles SET created_at = ? WHERE name IN (?, ?)",
                (_fmt(today - timedelta(days=61)), *(name for name, _ in BASE_VEHICLES)))
    return [tuple(r) for r in con.execute("SELECT id, owner_id FROM vehicles ORDER BY id")]


def _sample_jobs(con, rng, uid, vehicles, today):
    statuses = [s for s, n in STATUS_COUNT.items() for _ in range(n)]
    rng.shuffle(statuses)
    manual = set(rng.sample([i for i, s in enumerate(statuses) if s != "no_damage"], N_MANUAL))
    u3 = [vid for vid, o in vehicles if o == uid["user3"]]                 # user3's vehicles have 2+ records
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
        # log times: spread between start and end in proportion to the step weights
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
    """Last 30 days 9-18 h, jobs never overlap (16+ minutes apart, jobs 15 minutes or less)"""
    per_day = {}
    for d in rng.choices(range(1, DAYS + 1), k=N_JOBS):
        per_day[d] = per_day.get(d, 0) + 1
    slots = []
    for d in sorted(per_day, reverse=True):                               # oldest day first → ids in time order
        n = per_day[d]
        t = (today - timedelta(days=d)).replace(hour=9) + timedelta(minutes=rng.randint(0, 40))
        for _ in range(n):
            slots.append(t)
            t += timedelta(minutes=rng.uniform(16, max(17, 440 / n)))
    return slots


class _Job:
    def __init__(self):
        self.steps = []          # (weight, stage, level, message, replacement row or None)
        self.findings = []       # (part_code, x, y, z, depth, decision)
        self.status = self.summary = None

    def ev(self, stage, message, level="info", weight=1.0, rep=None):
        self.steps.append((weight, stage, level, message, rep))


def _targets(rng, status):
    """Parts to replace: only robot-replaceable (robot_ok=1) parts - bonnet, mirrors, doors"""
    if status == "no_damage":
        return []
    if status == "on_hold":                                               # always includes the out-of-stock side mirror (R)
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
    stop_at = rng.choice(["scan", "replace"] if mode == "auto" else ["replace"]) \
        if status == "stopped" else None

    def stopped():
        j.status, j.summary = "stopped", "관리자가 정지 버튼을 누름"
        return j

    j.ev("start", "수리 시작 (자동 모드)" if mode == "auto" else f"수리 시작 (수동 모드: {', '.join(targets)})", weight=0)
    if mode == "manual":
        j.findings += [(c, None, None, None, None, "requested") for c in targets]

    if mode == "auto":
        dents = []
        if "bonnet" in targets:
            dents += [("bonnet", round(rng.uniform(*BONNET_X), 1), round(rng.uniform(*BONNET_Y), 1), None)
                      for _ in range(rng.choice([1, 1, 1, 2]))]
        for d in (c for c in targets if c in DOOR_X):
            dents.append((d, round(rng.uniform(*DOOR_X[d]), 1), DOOR_Y[d], round(rng.uniform(*DOOR_Z), 1)))
        j.ev("scan", f"표면 스캔(연속 접촉) 시작 — 윗면 {N_TOP_AREAS}구역, 라인 간격 15 mm, 점 간격 5 mm", weight=1 if dents else 4)
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

    lacking = []                                                          # past stock is assumed to differ from now; only on-hold jobs have 0 stock
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
    """Start to end [min]: 3-15 minutes depending on the number of replaced parts"""
    n_rep = sum(1 for s in j.steps if s[4])
    if j.status in ("no_damage", "on_hold", "stopped") or (j.status == "failed" and n_rep == 0):
        return rng.uniform(3, 8)
    return rng.uniform(*{0: (3, 7), 1: (6, 11)}.get(n_rep, (10, 15)))
