"""
자동차 자동 수리 로봇 - 웹 서버 (Flask + SQLite)

실행:  python3 app.py            → http://<PC IP>:5000  (태블릿에서 접속)
처음 실행 시 repair.db 를 만들고 기본 계정·부품·차량을 넣는다.
  관리자 admin / admin1234,  사용자(차주) user1 / user1234, user2 / user1234   ← 꼭 바꾸기

권한
  admin : 수리 실행(자동/수동), 재고, 회원·차량 관리, 전체 이력
  user  : 본인 차량의 수리 현황(3D)과 이력만 조회
  robot : ROS 노드. HTTP 헤더 X-API-Key (환경변수 REPAIR_API_KEY, 기본 'rokey-robot-key')
"""
import os
import sqlite3
from functools import wraps

from flask import Flask, abort, jsonify, redirect, render_template, request, session, url_for
from werkzeug.security import check_password_hash, generate_password_hash

import db

app = Flask(__name__)
app.secret_key = os.environ.get("REPAIR_SECRET", "change-this-secret")
API_KEY = os.environ.get("REPAIR_API_KEY", "rokey-robot-key")
ROSBRIDGE_URL = os.environ.get("ROSBRIDGE_URL", "")   # 비우면 브라우저가 ws://<접속한 호스트>:9090 사용
app.teardown_appcontext(db.close_db)

FINISH_STATUSES = {"done", "no_damage", "on_hold", "failed", "stopped"}
MODES = {"auto", "manual"}


# ================================================================ 인증 / 권한
def role():
    if request.headers.get("X-API-Key") == API_KEY:
        return "robot"
    return session.get("role")


def require(*roles):
    """require('admin', 'robot') 처럼 허용할 권한을 적는다. 인자 없으면 로그인만 확인."""
    def deco(fn):
        @wraps(fn)
        def wrapper(*a, **kw):
            r = role()
            if r is None:
                if request.path.startswith("/api/"):
                    return jsonify(error="로그인이 필요합니다"), 401
                return redirect(url_for("login"))
            if roles and r not in roles:
                if request.path.startswith("/api/"):
                    return jsonify(error="권한이 없습니다"), 403
                return redirect(url_for("home"))
            return fn(*a, **kw)
        return wrapper
    return deco


def can_see_vehicle(vehicle_id):
    if role() in ("admin", "robot"):
        return True
    v = db.get_db().execute("SELECT owner_id FROM vehicles WHERE id = ?", (vehicle_id,)).fetchone()
    return v is not None and v["owner_id"] == session.get("user_id")


def job_or_404(job_id):
    j = db.get_db().execute("SELECT * FROM jobs WHERE id = ?", (job_id,)).fetchone()
    if not j or not can_see_vehicle(j["vehicle_id"]):
        abort(404)   # 남의 차 작업은 '없음'으로 응답
    return j


@app.errorhandler(404)
def not_found(_):
    if request.path.startswith("/api/"):
        return jsonify(error="찾을 수 없습니다"), 404
    return redirect(url_for("home"))


# ================================================================ 페이지
@app.route("/login", methods=["GET", "POST"])
def login():
    error = None
    if request.method == "POST":
        u = db.get_db().execute("SELECT * FROM users WHERE username = ?",
                                (request.form.get("username", "").strip(),)).fetchone()
        if u and check_password_hash(u["pw_hash"], request.form.get("password", "")):
            session.clear()
            session.update(user_id=u["id"], user=u["username"], name=u["name"] or u["username"], role=u["role"])
            return redirect(url_for("home"))
        error = "아이디 또는 비밀번호가 맞지 않습니다."
    return render_template("login.html", error=error)


@app.route("/logout")
def logout():
    session.clear()
    return redirect(url_for("login"))


@app.route("/")
@require()
def home():
    return redirect(url_for("admin_page") if role() == "admin" else url_for("my_page"))


@app.route("/admin")
@require("admin")
def admin_page():
    return render_template("admin.html", name=session["name"], rosbridge_url=ROSBRIDGE_URL)


@app.route("/my")
@require("user")
def my_page():
    return render_template("my.html", name=session["name"])


# ================================================================ 회원 (관리자)
@app.get("/api/me")
@require()
def me():
    return jsonify(id=session.get("user_id"), username=session.get("user"), name=session.get("name"), role=role())


@app.get("/api/users")
@require("admin")
def list_users():
    return jsonify(db.rows(db.get_db().execute(
        "SELECT id, username, name, role FROM users ORDER BY role, id")))


@app.post("/api/users")
@require("admin")
def add_user():
    d = request.get_json(force=True)
    username, pw = (d.get("username") or "").strip(), d.get("password") or ""
    r = d.get("role", "user")
    if not username or len(pw) < 4 or r not in ("admin", "user"):
        return jsonify(error="아이디, 4자 이상 비밀번호, 권한(admin/user)을 확인하세요"), 400
    con = db.get_db()
    try:
        cur = con.execute("INSERT INTO users (username, pw_hash, name, role) VALUES (?, ?, ?, ?)",
                          (username, generate_password_hash(pw), (d.get("name") or username).strip(), r))
        con.commit()
    except sqlite3.IntegrityError:
        return jsonify(error="이미 있는 아이디입니다"), 409
    return jsonify(id=cur.lastrowid, username=username, role=r), 201


# ================================================================ 차량
VEHICLE_SQL = """SELECT v.*, u.name AS owner_name, u.username AS owner_username
                 FROM vehicles v LEFT JOIN users u ON u.id = v.owner_id"""


@app.get("/api/vehicles")
@require()
def list_vehicles():
    con = db.get_db()
    if role() == "user":
        cur = con.execute(VEHICLE_SQL + " WHERE v.owner_id = ? ORDER BY v.id", (session["user_id"],))
    else:
        cur = con.execute(VEHICLE_SQL + " ORDER BY v.id")
    return jsonify(db.rows(cur))


@app.post("/api/vehicles")
@require("admin")
def add_vehicle():
    d = request.get_json(force=True)
    name = (d.get("name") or "").strip()
    if not name:
        return jsonify(error="차량 이름을 입력하세요"), 400
    owner = d.get("owner_id") or None
    con = db.get_db()
    if owner and not con.execute("SELECT 1 FROM users WHERE id = ? AND role = 'user'", (owner,)).fetchone():
        return jsonify(error="차주는 사용자 계정이어야 합니다"), 400
    try:
        cur = con.execute("INSERT INTO vehicles (name, model, owner_id) VALUES (?, ?, ?)",
                          (name, d.get("model") or "sedan_1_18", owner))
        con.commit()
    except sqlite3.IntegrityError:
        return jsonify(error="이미 등록된 차량 이름입니다"), 409
    return jsonify(dict(con.execute(VEHICLE_SQL + " WHERE v.id = ?", (cur.lastrowid,)).fetchone())), 201


@app.patch("/api/vehicles/<int:vid>")
@require("admin")
def set_vehicle_owner(vid):
    d = request.get_json(force=True)
    con = db.get_db()
    cur = con.execute("UPDATE vehicles SET owner_id = ? WHERE id = ?", (d.get("owner_id") or None, vid))
    con.commit()
    if cur.rowcount == 0:
        abort(404)
    return jsonify(dict(con.execute(VEHICLE_SQL + " WHERE v.id = ?", (vid,)).fetchone()))


# ================================================================ 부품 재고
@app.get("/api/parts")
@require("admin", "robot")
def list_parts():
    return jsonify(db.rows(db.get_db().execute("SELECT * FROM parts ORDER BY code")))


@app.get("/api/parts/<code>/check")
@require("admin", "robot")
def check_part(code):
    p = db.get_db().execute("SELECT * FROM parts WHERE code = ?", (code,)).fetchone()
    if not p:
        abort(404)
    return jsonify(code=code, name=p["name"], stock=p["stock"], available=p["stock"] > 0)


@app.post("/api/parts/<code>/consume")
@require("admin", "robot")
def consume_part(code):
    """교체 성공 후 재고 1개 차감. 재고가 없으면 409"""
    con = db.get_db()
    cur = con.execute("UPDATE parts SET stock = stock - 1 WHERE code = ? AND stock > 0", (code,))
    con.commit()
    if cur.rowcount == 0:
        return jsonify(error="재고 없음 또는 없는 부품", code=code), 409
    return jsonify(dict(con.execute("SELECT * FROM parts WHERE code = ?", (code,)).fetchone()))


@app.patch("/api/parts/<code>")
@require("admin")
def set_part(code):
    stock = request.get_json(force=True).get("stock")
    if not isinstance(stock, int) or stock < 0:
        return jsonify(error="재고는 0 이상의 정수"), 400
    con = db.get_db()
    cur = con.execute("UPDATE parts SET stock = ? WHERE code = ?", (stock, code))
    con.commit()
    if cur.rowcount == 0:
        abort(404)
    return jsonify(dict(con.execute("SELECT * FROM parts WHERE code = ?", (code,)).fetchone()))


# ================================================================ 수리 작업
JOB_SQL = """SELECT j.*, v.name AS vehicle_name, v.model, v.owner_id, u.name AS operator_name,
                    (SELECT group_concat(part_code, ',') FROM replacements r
                      WHERE r.job_id = j.id AND r.success = 1) AS replaced
             FROM jobs j JOIN vehicles v ON v.id = j.vehicle_id
                         JOIN users u ON u.id = j.operator_id"""


def job_detail(job_id):
    con = db.get_db()
    j = con.execute(JOB_SQL + " WHERE j.id = ?", (job_id,)).fetchone()
    if not j:
        return None
    d = dict(j)
    d["findings"] = db.rows(con.execute("SELECT * FROM findings WHERE job_id = ? ORDER BY id", (job_id,)))
    d["replacements"] = db.rows(con.execute("SELECT * FROM replacements WHERE job_id = ? ORDER BY id", (job_id,)))
    d["events"] = db.rows(con.execute("SELECT * FROM events WHERE job_id = ? ORDER BY id", (job_id,)))
    d["requested"] = [f["part_code"] for f in d["findings"] if f["decision"] == "requested"]
    return d


@app.post("/api/jobs")
@require("admin", "robot")
def create_job():
    """수리 시작. {vehicle_id, mode: auto|manual, parts: [수동일 때 교체할 부품 코드]}"""
    d = request.get_json(force=True)
    con = db.get_db()
    mode = d.get("mode", "auto")
    if mode not in MODES:
        return jsonify(error="mode 는 auto 또는 manual"), 400
    if not con.execute("SELECT 1 FROM vehicles WHERE id = ?", (d.get("vehicle_id"),)).fetchone():
        return jsonify(error="없는 차량입니다"), 400
    parts = []
    if mode == "manual":
        valid = {r["code"] for r in con.execute("SELECT code FROM parts")}
        parts = [p for p in dict.fromkeys(d.get("parts") or []) if p in valid]
        if not parts:
            return jsonify(error="수동 모드는 교체할 부품을 1개 이상 고르세요"), 400
    if con.execute("SELECT 1 FROM jobs WHERE status = 'running'").fetchone():
        return jsonify(error="이미 진행 중인 수리가 있습니다"), 409
    if role() == "robot":
        op = d.get("operator_id") or con.execute("SELECT id FROM users WHERE role = 'admin' ORDER BY id").fetchone()["id"]
    else:
        op = session["user_id"]
    cur = con.execute("INSERT INTO jobs (vehicle_id, operator_id, mode) VALUES (?, ?, ?)", (d["vehicle_id"], op, mode))
    jid = cur.lastrowid
    con.executemany("INSERT INTO findings (job_id, part_code, decision) VALUES (?, ?, 'requested')",
                    [(jid, p) for p in parts])
    msg = "수리 시작 (자동 모드)" if mode == "auto" else f"수리 시작 (수동 모드: {', '.join(parts)})"
    con.execute("INSERT INTO events (job_id, stage, message) VALUES (?, 'start', ?)", (jid, msg))
    con.commit()
    return jsonify(job_detail(jid)), 201


@app.get("/api/jobs")
@require()
def list_jobs():
    limit = min(int(request.args.get("limit", 50)), 500)
    where, args = [], []
    if role() == "user":
        where.append("v.owner_id = ?"); args.append(session["user_id"])
    if request.args.get("vehicle_id"):
        where.append("j.vehicle_id = ?"); args.append(int(request.args["vehicle_id"]))
    q = JOB_SQL + (" WHERE " + " AND ".join(where) if where else "") + " ORDER BY j.id DESC LIMIT ?"
    return jsonify(db.rows(db.get_db().execute(q, (*args, limit))))


@app.get("/api/jobs/current")
@require()
def current_job():
    """진행 중인 작업. 사용자는 본인 차량 것만 보인다. ?vehicle_id= 로 차량 지정 가능"""
    where, args = ["j.status = 'running'"], []
    if role() == "user":
        where.append("v.owner_id = ?"); args.append(session["user_id"])
    if request.args.get("vehicle_id"):
        where.append("j.vehicle_id = ?"); args.append(int(request.args["vehicle_id"]))
    j = db.get_db().execute(JOB_SQL + " WHERE " + " AND ".join(where) + " ORDER BY j.id DESC LIMIT 1", args).fetchone()
    return jsonify(job_detail(j["id"]) if j else None)


@app.get("/api/jobs/<int:job_id>")
@require()
def get_job(job_id):
    job_or_404(job_id)
    return jsonify(job_detail(job_id))


@app.post("/api/jobs/<int:job_id>/events")
@require("admin", "robot")
def add_event(job_id):
    job_or_404(job_id)
    d = request.get_json(force=True)
    con = db.get_db()
    con.execute("INSERT INTO events (job_id, stage, level, message) VALUES (?, ?, ?, ?)",
                (job_id, d.get("stage", "info"), d.get("level", "info"), d.get("message", "")))
    con.commit()
    return jsonify(ok=True), 201


@app.post("/api/jobs/<int:job_id>/findings")
@require("admin", "robot")
def add_findings(job_id):
    job_or_404(job_id)
    items = request.get_json(force=True)
    items = items if isinstance(items, list) else [items]
    con = db.get_db()
    try:
        con.executemany("INSERT INTO findings (job_id, part_code, x, y, depth, decision) VALUES (?, ?, ?, ?, ?, ?)",
                        [(job_id, f["part_code"], f.get("x"), f.get("y"), f.get("depth"), f.get("decision", "dent"))
                         for f in items])
        con.commit()
    except (sqlite3.IntegrityError, KeyError) as e:
        return jsonify(error=f"잘못된 판정 데이터: {e}"), 400
    return jsonify(ok=True, count=len(items)), 201


@app.post("/api/jobs/<int:job_id>/replacements")
@require("admin", "robot")
def add_replacement(job_id):
    job_or_404(job_id)
    d = request.get_json(force=True)
    con = db.get_db()
    try:
        con.execute("""INSERT INTO replacements (job_id, part_code, success, retries, peak_force, flush_mm)
                       VALUES (?, ?, ?, ?, ?, ?)""",
                    (job_id, d["part_code"], int(bool(d.get("success"))), d.get("retries", 0),
                     d.get("peak_force"), d.get("flush_mm")))
        con.commit()
    except (sqlite3.IntegrityError, KeyError) as e:
        return jsonify(error=f"잘못된 교체 데이터: {e}"), 400
    return jsonify(ok=True), 201


@app.post("/api/jobs/<int:job_id>/finish")
@require("admin", "robot")
def finish_job(job_id):
    j = job_or_404(job_id)
    d = request.get_json(force=True)
    status = d.get("status", "done")
    if status not in FINISH_STATUSES:
        return jsonify(error=f"status 는 {sorted(FINISH_STATUSES)} 중 하나"), 400
    if j["status"] != "running":
        return jsonify(job_detail(job_id))   # 이미 끝난 작업은 그대로 (정지 버튼과 로봇 종료가 겹칠 때)
    con = db.get_db()
    con.execute("UPDATE jobs SET status = ?, summary = ?, finished_at = datetime('now','localtime') WHERE id = ?",
                (status, d.get("summary"), job_id))
    con.execute("INSERT INTO events (job_id, stage, level, message) VALUES (?, 'finish', ?, ?)",
                (job_id, "info" if status in ("done", "no_damage") else "warn", d.get("summary") or status))
    con.commit()
    return jsonify(job_detail(job_id))


if __name__ == "__main__":
    db.init_db()
    app.run(host="0.0.0.0", port=int(os.environ.get("PORT", 5000)), debug=False)
