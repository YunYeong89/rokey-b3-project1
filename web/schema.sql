-- 자동차 자동 수리 로봇 DB 스키마 (SQLite3)

CREATE TABLE IF NOT EXISTS users (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    username    TEXT UNIQUE NOT NULL,
    pw_hash     TEXT NOT NULL,
    name        TEXT,                                 -- 표시 이름 (예: 홍길동)
    role        TEXT NOT NULL DEFAULT 'user'          -- admin(관리자, 정비소) / user(사용자, 차주)
                CHECK (role IN ('admin', 'user'))
);

-- 수리 대상 차량 (축소 모형)
CREATE TABLE IF NOT EXISTS vehicles (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    name        TEXT UNIQUE NOT NULL,                 -- 예: 모형 A-01 (번호판 역할)
    model       TEXT NOT NULL DEFAULT 'sedan_1_18',   -- 기준 모델 이름 (정상 높이 표와 연결)
    owner_id    INTEGER REFERENCES users(id),         -- 차주 (사용자 계정)
    created_at  TEXT NOT NULL DEFAULT (datetime('now', 'localtime'))
);

-- 교체 부품 재고
CREATE TABLE IF NOT EXISTS parts (
    code        TEXT PRIMARY KEY,                     -- bonnet, mirror_l, mirror_r
    name        TEXT NOT NULL,
    stock       INTEGER NOT NULL DEFAULT 0 CHECK (stock >= 0),
    min_stock   INTEGER NOT NULL DEFAULT 1            -- 이 이하이면 '부족' 표시
);

-- 수리 작업 1건
CREATE TABLE IF NOT EXISTS jobs (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    vehicle_id  INTEGER NOT NULL REFERENCES vehicles(id),
    operator_id INTEGER NOT NULL REFERENCES users(id),  -- 실행한 관리자
    mode        TEXT NOT NULL DEFAULT 'auto'          -- auto(스캔으로 손상 추적) / manual(지정 부품만 교체)
                CHECK (mode IN ('auto', 'manual')),
    status      TEXT NOT NULL DEFAULT 'running',      -- running / done / no_damage / on_hold / failed / stopped
    summary     TEXT,
    started_at  TEXT NOT NULL DEFAULT (datetime('now', 'localtime')),
    finished_at TEXT
);

-- 손상 판정 결과 (손상 점 단위)
CREATE TABLE IF NOT EXISTS findings (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    job_id      INTEGER NOT NULL REFERENCES jobs(id),
    part_code   TEXT NOT NULL REFERENCES parts(code),
    x           REAL,                                 -- 차량 좌표계 [mm]
    y           REAL,
    depth       REAL,                                 -- 기준 대비 깊이 [mm], 부품 없음이면 NULL
    decision    TEXT NOT NULL                         -- dent / missing / suspect / requested(수동 모드에서 관리자가 지정)
);

-- 부품 교체 결과
CREATE TABLE IF NOT EXISTS replacements (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    job_id      INTEGER NOT NULL REFERENCES jobs(id),
    part_code   TEXT NOT NULL REFERENCES parts(code),
    success     INTEGER NOT NULL,                     -- 1 / 0
    retries     INTEGER NOT NULL DEFAULT 0,
    peak_force  REAL,                                 -- 장착 시 최대 힘 [N]
    flush_mm    REAL,                                 -- 재검사 단차 [mm]
    ts          TEXT NOT NULL DEFAULT (datetime('now', 'localtime'))
);

-- 진행 로그 (단계별 이벤트, 오류)
CREATE TABLE IF NOT EXISTS events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    job_id      INTEGER NOT NULL REFERENCES jobs(id),
    ts          TEXT NOT NULL DEFAULT (datetime('now', 'localtime')),
    stage       TEXT NOT NULL,                        -- start / calibrate / scan / detect / stock / replace / inspect / done / finish
    level       TEXT NOT NULL DEFAULT 'info',         -- info / warn / error
    message     TEXT NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_events_job ON events(job_id);
CREATE INDEX IF NOT EXISTS idx_findings_job ON findings(job_id);
CREATE INDEX IF NOT EXISTS idx_jobs_vehicle ON jobs(vehicle_id);
