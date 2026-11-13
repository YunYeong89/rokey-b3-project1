"""SQLite 연결과 초기화"""
import os
import sqlite3

from flask import g
from werkzeug.security import generate_password_hash

BASE = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.environ.get("REPAIR_DB", os.path.join(BASE, "repair.db"))


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
    with open(os.path.join(BASE, "schema.sql"), encoding="utf-8") as f:
        con.executescript(f.read())
    if seed and con.execute("SELECT COUNT(*) FROM users").fetchone()[0] == 0:
        # 기본 계정 (처음 실행 후 비밀번호 바꾸기)
        users = [("admin", "admin1234", "정비소 관리자", "admin"),
                 ("user1", "user1234", "차주 1", "user"),
                 ("user2", "user1234", "차주 2", "user")]
        for u, pw, name, role in users:
            con.execute("INSERT INTO users (username, pw_hash, name, role) VALUES (?, ?, ?, ?)",
                        (u, generate_password_hash(pw), name, role))
        con.executemany("INSERT OR IGNORE INTO parts (code, name, stock, min_stock) VALUES (?, ?, ?, ?)", [
            ("bonnet", "보닛", 3, 1),
            ("mirror_l", "사이드미러 (좌)", 2, 1),
            ("mirror_r", "사이드미러 (우)", 2, 1),
        ])
        uid = {r[0]: r[1] for r in con.execute("SELECT username, id FROM users")}
        con.executemany("INSERT OR IGNORE INTO vehicles (name, model, owner_id) VALUES (?, ?, ?)", [
            ("모형 A-01", "sedan_1_18", uid["user1"]),
            ("모형 A-02", "sedan_1_18", uid["user2"]),
        ])
    con.commit()
    con.close()


def rows(cur):
    return [dict(r) for r in cur.fetchall()]
