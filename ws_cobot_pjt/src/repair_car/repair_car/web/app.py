"""
Scan&Fix (automatic car repair robot) - web server (Flask + SQLite)

Run:  python3 app.py            → http://<PC IP>:5050  (open it on a tablet)
The first run creates repair.db with the default accounts, parts and vehicles + sample data (members, vehicles, history).
  Without samples: REPAIR_SAMPLE=0 python3 app.py      Fresh start: rm repair.db, then run again
  admin admin / admin1234,  users (owners) user1 / user1234, user2 / user1234   ← change these

Roles
  admin : run repairs (auto/manual), stock, member and vehicle management, full history
  user  : only the repair status (3D) and history of their own vehicles
  robot : ROS nodes. HTTP header X-API-Key (environment variable REPAIR_API_KEY, default 'rokey-robot-key')

Owner notification mail (repair start · end). Owner emails are entered in the admin 'Members' tab
  SMTP_USER=sender Gmail  SMTP_PASS=Gmail app password  SITE_URL=http://<PC IP>:5050  python3 app.py
  To send with Naver: add SMTP_HOST=smtp.naver.com (turn on POP3/SMTP in the Naver mail settings)
  MAIL_DRY_RUN=1 only prints the mail to the terminal instead of sending it
  The values above can be written as KEY='value' lines in web/.env, read by plain python3 app.py (.env is git-ignored).
  Non-empty environment variables given in the terminal take precedence over .env.
"""
import json
import logging
import math
import html
import os
import secrets
import smtplib
import sqlite3
import threading
from pathlib import Path
from email.message import EmailMessage
from email.utils import formataddr
from functools import wraps

from itsdangerous import BadSignature, URLSafeTimedSerializer

from flask import Flask, abort, jsonify, redirect, render_template, request, send_file, session, url_for
from werkzeug.security import check_password_hash, generate_password_hash


def load_env_file(path=Path(__file__).resolve().parents[2] / "config/web.env"):
    """Read config/web.env without replacing values supplied by the launcher."""
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, val = line.removeprefix("export ").split("=", 1)
                key, val = key.strip(), val.strip()
                if len(val) >= 2 and val[0] == val[-1] and val[0] in "'\"":
                    val = val[1:-1]
                if not os.environ.get(key):
                    os.environ[key] = val
    except FileNotFoundError:
        pass


load_env_file()          # before import db (db also reads environment variables such as REPAIR_DB)

if __package__:
    from . import db
    from .robot_files import RobotFiles
else:
    import db
    from robot_files import RobotFiles

app = Flask(__name__)


def _secret_key():
    """REPAIR_SECRET, else a random key kept in config/web.secret_key (git-ignored): the old fixed default was public in the
    repository, so anyone could forge a login cookie or a mail login link."""
    if os.environ.get("REPAIR_SECRET"):
        return os.environ["REPAIR_SECRET"]
    path = Path(__file__).resolve().parents[2] / "config/web.secret_key"
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with open(path) as f:
            return f.read().strip()
    except FileNotFoundError:
        key = secrets.token_hex(32)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as f:
            f.write(key)
        return key


app.secret_key = _secret_key()
app.config["TEMPLATES_AUTO_RELOAD"] = True   # templates/*.html edits apply on reload even with debug=False
API_KEY = os.environ.get("REPAIR_API_KEY", "rokey-robot-key")
ROSBRIDGE_URL = os.environ.get("ROSBRIDGE_URL", "")   # empty: the browser uses ws://<connected host>:ROSBRIDGE_PORT
ROSBRIDGE_PORT = int(os.environ.get("ROSBRIDGE_PORT", 9090))   # run_web.sh --lan: 9091 (lan_relay.py)
ROSBRIDGE_TLS_PORT = int(os.environ.get("ROSBRIDGE_TLS_PORT", 9443))   # wss for https pages (lan_relay.py TLS)
app.teardown_appcontext(db.close_db)

# service name (page title · mail subject). Can be changed with the SERVICE_NAME environment variable
SERVICE_NAME = os.environ.get("SERVICE_NAME", "Scan&Fix")
SERVICE_TAGLINE = "로봇이 스캔하고, 판정하고, 교체합니다"


@app.context_processor
def inject_service():
    return {"service_name": SERVICE_NAME, "service_tagline": SERVICE_TAGLINE}

FINISH_STATUSES = {"done", "no_damage", "on_hold", "failed", "stopped"}
MODES = {"auto", "manual"}
INSPECTION_PARTS = ("bonnet", "left_door", "right_door")


# ================================================================ owner notification mail
SMTP_USER = os.environ.get("SMTP_USER", "")
SMTP_PASS = os.environ.get("SMTP_PASS", "")        # never logged
SMTP_HOST = os.environ.get("SMTP_HOST", "smtp.gmail.com")   # Naver: smtp.naver.com
SMTP_PORT = int(os.environ.get("SMTP_PORT", "465"))         # SSL
SITE_URL = os.environ.get("SITE_URL", "").rstrip("/")
MAIL_DRY_RUN = os.environ.get("MAIL_DRY_RUN") == "1"
MAIL_LINK_DAYS = int(os.environ.get("MAIL_LINK_DAYS", "7"))   # validity of the mail's "View result" direct login link

mail_log = logging.getLogger("repair.mail")
mail_log.setLevel(logging.INFO)
if not mail_log.handlers:
    _h = logging.StreamHandler()
    _h.setFormatter(logging.Formatter("[mail %(asctime)s] %(message)s", "%H:%M:%S"))
    mail_log.addHandler(_h)
    mail_log.propagate = False

FINISH_TEXT = {
    "done": "수리가 완료되었습니다",
    "no_damage": "점검 결과 손상이 없습니다",
    "on_hold": "부품 재고 부족으로 수리가 보류되었습니다",
    "failed": "수리가 중단되었습니다. 담당자가 확인 중입니다",
    "stopped": "관리자가 수리를 중지했습니다",
}


def valid_email(s):
    """Simple format check: no spaces, characters before and after @, a dot after it"""
    local, at, domain = s.partition("@")
    return bool(at and local and "." in domain.strip(".") and " " not in s and len(s) <= 254)


def mask_email(s):
    """For logs: ab***@gmail.com"""
    local, _, domain = (s or "").partition("@")
    return f"{local[:2]}***@{domain}" if domain else "(none)"


def notify_owner(con, job_id, kind):
    """kind = 'start' or an end state (done, stopped …). Builds the content inside the request and sends it on a thread.
    Never raises on failure (must not affect the repair)"""
    try:
        msg = _build_mail(con, job_id, kind)
    except Exception as e:                                   # the repair API must not fail because of mail
        mail_log.warning("Job #%s: building the mail failed: %s", job_id, type(e).__name__)
        return
    if msg is None:
        return
    if not MAIL_DRY_RUN and not (SMTP_USER and SMTP_PASS):
        mail_log.info("Job #%s mail skipped: SMTP_USER / SMTP_PASS not set", job_id)
        return
    threading.Thread(target=_send_mail, args=(job_id, kind, msg), daemon=True).start()


def _build_mail(con, job_id, kind):
    j = con.execute("""SELECT j.mode, j.status, j.summary, j.started_at, j.finished_at, v.name AS vehicle,
                              v.owner_id, u.name AS owner_name, u.email
                       FROM jobs j JOIN vehicles v ON v.id = j.vehicle_id LEFT JOIN users u ON u.id = v.owner_id
                       WHERE j.id = ?""", (job_id,)).fetchone()
    if j is None or j["owner_id"] is None:
        mail_log.info("Job #%s mail skipped: vehicle without an owner", job_id)
        return None
    if not j["email"]:
        mail_log.info("Job #%s mail skipped: owner has no email", job_id)
        return None

    names = lambda sql: [r[0] for r in con.execute(sql, (job_id,))]
    hello = f"{j['owner_name'] or '차주'} 님, 차량 {j['vehicle']}"
    if kind == "start":
        subject = f"[{SERVICE_NAME}] {j['vehicle']} 수리를 시작했습니다"
        if j["mode"] == "manual":
            parts = names("""SELECT p.name FROM findings f JOIN parts p ON p.code = f.part_code
                             WHERE f.job_id = ? AND f.decision = 'requested' ORDER BY f.id""")
            mode = f"수동 (교체 부품: {', '.join(parts)})"
        else:
            mode = "자동 (표면 스캔으로 손상 부위를 찾아 교체)"
        lines = [f"{hello} 수리를 시작했습니다.", "", f"시작 시각: {j['started_at']}", f"모드: {mode}", "",
                 "진행 상황은 아래 링크에서 실시간으로 볼 수 있습니다."]
    else:
        text = FINISH_TEXT[kind]
        subject = f"[{SERVICE_NAME}] {j['vehicle']} {text.split('.')[0]}"
        lines = [f"{hello}: {text}.", "", f"종료 시각: {j['finished_at']}"]
        if kind == "done":
            parts = names("""SELECT p.name FROM replacements r JOIN parts p ON p.code = r.part_code
                             WHERE r.job_id = ? AND r.success = 1 ORDER BY r.id""")
            lines.append(f"교체한 부품: {', '.join(dict.fromkeys(parts))}" if parts else f"내용: {j['summary'] or '-'}")
        elif kind == "on_hold" and j["summary"]:
            lines.append(f"사유: {j['summary']}")
    link = f"{SITE_URL}/m/{_mail_links().dumps({'u': j['owner_id'], 'j': job_id})}" if SITE_URL else None
    if link:
        lines += ["", f"결과 보기 (로그인 없이 바로 열림, {MAIL_LINK_DAYS}일 동안): {link}"]

    msg = EmailMessage()
    msg["Subject"] = subject
    msg["From"] = formataddr((SERVICE_NAME, SMTP_USER or "dry-run@localhost"))
    msg["To"] = j["email"]
    msg.set_content("\n".join(lines))
    # the same content as HTML: the "View result" button opens my vehicle page directly (text-only mail clients use the link above)
    body = "".join(f"<p style='margin:0 0 6px'>{html.escape(line)}</p>" if line else "<br>"
                   for line in lines if not line.startswith("결과 보기"))
    if link:
        body += (f"<p style='margin:18px 0'><a href='{html.escape(link)}' style='background:#2f6fd6;color:#fff;"
                 "padding:10px 18px;border-radius:6px;text-decoration:none;font-weight:600'>결과 보기</a></p>"
                 f"<p style='color:#777;font-size:12px'>로그인 없이 바로 열립니다 ({MAIL_LINK_DAYS}일 동안 유효). "
                 "다른 사람에게 전달하지 마세요.</p>")
    msg.add_alternative(f"<div style='font-family:sans-serif;font-size:14px;line-height:1.6'>{body}</div>",
                        subtype="html")
    return msg


def _mail_links():
    """Signed, expiring login links for the owner (secret_key), so the mail's button opens the page directly."""
    return URLSafeTimedSerializer(app.secret_key, salt="mail-login")


MAIL_LABEL = {"start": "수리 시작 알림"}


def _send_mail(job_id, kind, msg):
    """Runs on a thread. Results go to the terminal log and events (stage 'notify')"""
    label = MAIL_LABEL.get(kind, f"수리 종료 알림({kind})")
    to = mask_email(msg["To"])
    try:
        if MAIL_DRY_RUN:
            mail_log.info("Job #%s [dry run, not sent] %s\n----\nTo: %s\nSubject: %s\n\n%s----",
                          job_id, label, to, msg["Subject"], msg.get_content())
            _notify_event(job_id, "info", f"메일 발송(시험 모드): {label}")
            return
        with smtplib.SMTP_SSL(SMTP_HOST, SMTP_PORT, timeout=10) as s:
            s.login(SMTP_USER, SMTP_PASS)
            s.send_message(msg)
        mail_log.info("Job #%s mail sent: %s → %s", job_id, label, to)
        _notify_event(job_id, "info", f"메일 발송: {label}")
    except Exception as e:
        hint = f" (check the {SMTP_HOST} ID/password / SMTP settings)" if isinstance(e, smtplib.SMTPAuthenticationError) else ""
        code = getattr(e, "smtp_code", "")
        mail_log.warning("Job #%s mail failed: %s → %s: %s %s%s", job_id, label, to, type(e).__name__, code, hint)
        _notify_event(job_id, "warn", f"메일 발송 실패: {label} ({type(e).__name__})")


def _notify_event(job_id, level, message):
    """Thread-only connection (the Flask g connection belongs to the request thread)"""
    try:
        con = sqlite3.connect(db.DB_PATH, timeout=10)
        with con:
            con.execute("INSERT INTO events (job_id, stage, level, message) VALUES (?, 'notify', ?, ?)",
                        (job_id, level, message))
        con.close()
    except sqlite3.Error as e:
        mail_log.warning("Job #%s: recording the mail failed: %s", job_id, e)


# ================================================================ authentication / roles
def role():
    if request.headers.get("X-API-Key") == API_KEY:
        return "robot"
    return session.get("role")


def require(*roles):
    """List the allowed roles, as in require('admin', 'robot'). Without arguments only login is checked."""
    def deco(fn):
        @wraps(fn)
        def wrapper(*a, **kw):
            r = role()
            if r is None:
                if request.path.startswith("/api/"):
                    return jsonify(error="로그인이 필요합니다"), 401
                return redirect(url_for("login", next=request.full_path))   # back to the original address (?tab=robot etc.) after login
            if roles and r not in roles:
                if request.path.startswith("/api/"):
                    return jsonify(error="권한이 없습니다"), 403
                return redirect(url_for("home"))
            return fn(*a, **kw)
        return wrapper
    return deco


# robot program result files · live force graph · point clouds (default path: this repository's ws_cobot_pjt/ws_cobot_pjt, run_web.sh passes it as an environment variable)
PACKAGE = Path(__file__).resolve().parents[2]
robot_files = RobotFiles(os.environ.get("ROKEY_DIR", str(PACKAGE)),
           os.environ.get("SESSION_DIR", str(PACKAGE / "data")),
           [*(d for d in os.environ.get("SCENE_DIRS", "").split(os.pathsep) if d),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "static/robot/scene")]   # fixed real vehicle mesh (last)
           )
robot_files.register(app, require("admin"))


def can_see_vehicle(vehicle_id):
    if role() in ("admin", "robot"):
        return True
    v = db.get_db().execute("SELECT owner_id FROM vehicles WHERE id = ?", (vehicle_id,)).fetchone()
    return v is not None and v["owner_id"] == session.get("user_id")


def job_or_404(job_id):
    j = db.get_db().execute("SELECT * FROM jobs WHERE id = ?", (job_id,)).fetchone()
    if not j or not can_see_vehicle(j["vehicle_id"]):
        abort(404)   # someone else's car job answers 'not found'
    return j


@app.errorhandler(404)
def not_found(_):
    if request.path.startswith("/api/"):
        return jsonify(error="찾을 수 없습니다"), 404
    return redirect(url_for("home"))


# ================================================================ pages
@app.get("/ca.crt")
def local_ca():
    """Local certificate authority of run_web.sh --lan. Installed and trusted once on an iPad etc., the https address opens without a warning."""
    path = os.environ.get("WEB_CA", "")
    if not os.path.isfile(path):
        abort(404)
    return send_file(path, mimetype="application/x-x509-ca-cert", download_name="rokey-web-ca.crt")


def ca_profile(path):
    """iOS/iPadOS configuration profile with the same certificate authority (.mobileconfig: Safari/AirDrop shows the install screen)."""
    import plistlib, ssl, uuid
    pem = open(path).read()
    der = ssl.PEM_cert_to_DER_cert(pem)
    ident = uuid.uuid5(uuid.NAMESPACE_URL, pem)
    certificate = {"PayloadType": "com.apple.security.root", "PayloadVersion": 1, "PayloadContent": der,
                   "PayloadCertificateFileName": "rokey-web-ca.cer", "PayloadDisplayName": "rokey web local CA",
                   "PayloadIdentifier": f"rokey.web.ca.{ident}.cert", "PayloadUUID": str(uuid.uuid5(ident, "cert"))}
    return plistlib.dumps({"PayloadType": "Configuration", "PayloadVersion": 1, "PayloadContent": [certificate],
                           "PayloadDisplayName": "rokey web local CA", "PayloadIdentifier": f"rokey.web.ca.{ident}",
                           "PayloadUUID": str(ident)})


@app.get("/ca.mobileconfig")
def local_ca_profile():
    path = os.environ.get("WEB_CA", "")
    if not os.path.isfile(path):
        abort(404)
    return ca_profile(path), 200, {"Content-Type": "application/x-apple-aspen-config",
                                   "Content-Disposition": "attachment; filename=rokey-web-ca.mobileconfig"}


@app.route("/login", methods=["GET", "POST"])
def login():
    error = None
    if request.method == "POST":
        u = db.get_db().execute("SELECT * FROM users WHERE username = ?",
                                (request.form.get("username", "").strip(),)).fetchone()
        if u and check_password_hash(u["pw_hash"], request.form.get("password", "")):
            session.clear()
            session.update(user_id=u["id"], user=u["username"], name=u["name"] or u["username"], role=u["role"])
            target = request.args.get("next", "")
            return redirect(target if target.startswith("/") and not target.startswith("//") else url_for("home"))
        error = "아이디 또는 비밀번호가 맞지 않습니다."
    return render_template("login.html", error=error)


@app.route("/m/<token>")
def mail_login(token):
    """Mail "View result" link: after checking the signature and validity, log in as that owner and open my vehicle page. Expired or forged: login page"""
    try:
        data = _mail_links().loads(token, max_age=MAIL_LINK_DAYS * 86400)
    except BadSignature:   # includes SignatureExpired
        return redirect(url_for("login", next="/my"))
    u = db.get_db().execute("SELECT * FROM users WHERE id = ?", (data.get("u"),)).fetchone()
    if u is None or u["role"] != "user":   # owner accounts only (admin rights are never given by link)
        return redirect(url_for("login", next="/my"))
    session.clear()
    session.update(user_id=u["id"], user=u["username"], name=u["name"] or u["username"], role=u["role"])
    return redirect(url_for("my_page"))


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
    # without ROBOT_JOINT_TOPIC the page picks the joint topic of the connected robot (virtual/real)
    return render_template("admin.html", name=session["name"], rosbridge_url=ROSBRIDGE_URL,
                           rosbridge_port=ROSBRIDGE_PORT, rosbridge_tls_port=ROSBRIDGE_TLS_PORT,
                           robot_joint_topic=os.environ.get("ROBOT_JOINT_TOPIC", ""))


@app.route("/my")
@require("user")
def my_page():
    return render_template("my.html", name=session["name"])


@app.route("/api/robot/inspection-plan", methods=("GET", "PUT"))
@require("admin")
def inspection_plan():
    con, user = db.get_db(), session["user_id"]
    if request.method == "GET":
        row = con.execute("SELECT plan FROM inspection_plans WHERE user_id = ?", (user,)).fetchone()
        return jsonify(json.loads(row["plan"]) if row else [
            dict(part=part, enabled=True, mode="full", regions=[]) for part in INSPECTION_PARTS])
    plan = request.get_json(silent=True)
    if (not isinstance(plan, list) or len(plan) != len(INSPECTION_PARTS) or
            not all(isinstance(item, dict) and set(item) == {"part", "enabled", "mode", "regions"} and
                    item["part"] in INSPECTION_PARTS and type(item["enabled"]) is bool and
                    item["mode"] in ("full", "partial") and isinstance(item["regions"], list) for item in plan) or
            len({item["part"] for item in plan}) != len(INSPECTION_PARTS)):
        return jsonify(error="검사 부품과 순서를 확인하세요"), 400
    for item in plan:
        for box in item["regions"]:
            if (not isinstance(box, list) or len(box) != 4 or
                    not all(type(v) in (int, float) and math.isfinite(v) for v in box) or
                    box[2] <= box[0] or box[3] <= box[1]):
                return jsonify(error="부분 탐색 좌표를 확인하세요"), 400
    with con:
        con.execute("INSERT INTO inspection_plans (user_id, plan) VALUES (?, ?) "
                    "ON CONFLICT (user_id) DO UPDATE SET plan = excluded.plan", (user, json.dumps(plan)))
    return jsonify(plan)


# ================================================================ members (admin)
@app.get("/api/me")
@require()
def me():
    return jsonify(id=session.get("user_id"), username=session.get("user"), name=session.get("name"), role=role())


@app.get("/api/users")
@require("admin")
def list_users():
    return jsonify(db.rows(db.get_db().execute(
        "SELECT id, username, name, role, email FROM users ORDER BY role, id")))


@app.post("/api/users")
@require("admin")
def add_user():
    d = request.get_json(force=True)
    username, pw = (d.get("username") or "").strip(), d.get("password") or ""
    r = d.get("role", "user")
    if not username or len(pw) < 4 or r not in ("admin", "user"):
        return jsonify(error="아이디, 4자 이상 비밀번호, 권한(admin/user)을 확인하세요"), 400
    email = (d.get("email") or "").strip() or None
    if email and not valid_email(email):
        return jsonify(error="이메일 형식을 확인하세요 (예: name@example.com)"), 400
    con = db.get_db()
    try:
        cur = con.execute("INSERT INTO users (username, pw_hash, name, role, email) VALUES (?, ?, ?, ?, ?)",
                          (username, generate_password_hash(pw), (d.get("name") or username).strip(), r, email))
        con.commit()
    except sqlite3.IntegrityError:
        return jsonify(error="이미 있는 아이디입니다"), 409
    return jsonify(id=cur.lastrowid, username=username, role=r, email=email), 201


@app.patch("/api/users/<int:user_id>")
@require("admin")
def update_user(user_id):
    """Edit {email}. An empty value clears the email (no notifications)"""
    d = request.get_json(force=True)
    if "email" not in d:
        return jsonify(error="바꿀 값(email)이 없습니다"), 400
    email = (d.get("email") or "").strip() or None
    if email and not valid_email(email):
        return jsonify(error="이메일 형식을 확인하세요 (예: name@example.com)"), 400
    con = db.get_db()
    if con.execute("UPDATE users SET email = ? WHERE id = ?", (email, user_id)).rowcount == 0:
        return jsonify(error="없는 회원입니다"), 404
    con.commit()
    return jsonify(id=user_id, email=email)


# ================================================================ vehicles
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


# ================================================================ parts stock
@app.get("/api/parts")
@require("admin", "robot")
def list_parts():
    return jsonify(db.rows(db.get_db().execute(
        "SELECT * FROM parts ORDER BY CASE category WHEN '외판' THEN 0 WHEN '미러' THEN 1 WHEN '램프' THEN 2 ELSE 3 END, "
        "location, code")))


@app.get("/api/parts/<code>/check")
@require("admin", "robot")
def check_part(code):
    p = db.get_db().execute("SELECT * FROM parts WHERE code = ?", (code,)).fetchone()
    if not p:
        abort(404)
    return jsonify(code=code, name=p["name"], stock=p["stock"], available=p["stock"] > 0, robot_ok=bool(p["robot_ok"]))


@app.post("/api/parts/<code>/consume")
@require("admin", "robot")
def consume_part(code):
    """Deduct one from stock after a successful replacement. 409 when out of stock"""
    con = db.get_db()
    cur = con.execute("UPDATE parts SET stock = stock - 1 WHERE code = ? AND stock > 0", (code,))
    con.commit()
    if cur.rowcount == 0:
        return jsonify(error="재고 없음 또는 없는 부품", code=code), 409
    return jsonify(dict(con.execute("SELECT * FROM parts WHERE code = ?", (code,)).fetchone()))


@app.patch("/api/parts/<code>")
@require("admin")
def set_part(code):
    """{stock: integer >= 0} and/or {robot_ok: 0/1/true/false}"""
    d = request.get_json(force=True)
    sets = {}
    if "stock" in d:
        if not isinstance(d["stock"], int) or isinstance(d["stock"], bool) or d["stock"] < 0:
            return jsonify(error="재고는 0 이상의 정수"), 400
        sets["stock"] = d["stock"]
    if "robot_ok" in d:
        if d["robot_ok"] not in (0, 1):            # True/False equal 0/1 too
            return jsonify(error="robot_ok 는 0 또는 1"), 400
        sets["robot_ok"] = int(d["robot_ok"])
    if not sets:
        return jsonify(error="stock 또는 robot_ok 를 보내세요"), 400
    con = db.get_db()
    cur = con.execute(f"UPDATE parts SET {', '.join(k + ' = ?' for k in sets)} WHERE code = ?", (*sets.values(), code))
    con.commit()
    if cur.rowcount == 0:
        abort(404)
    return jsonify(dict(con.execute("SELECT * FROM parts WHERE code = ?", (code,)).fetchone()))


# ================================================================ repair jobs
JOB_SQL = """SELECT j.*, v.name AS vehicle_name, v.model, v.owner_id, v.robot_namespace, u.name AS operator_name,
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
    """Start a repair. {vehicle_id, mode: auto|manual, parts: [part codes to replace in manual mode]}"""
    d = request.get_json(force=True)
    con = db.get_db()
    mode = d.get("mode", "auto")
    if mode not in MODES:
        return jsonify(error="mode 는 auto 또는 manual"), 400
    if not con.execute("SELECT 1 FROM vehicles WHERE id = ?", (d.get("vehicle_id"),)).fetchone():
        return jsonify(error="없는 차량입니다"), 400
    parts = []
    if mode == "manual":
        valid = {r["code"]: r for r in con.execute("SELECT code, name, robot_ok FROM parts")}
        parts = [p for p in dict.fromkeys(d.get("parts") or []) if p in valid]
        if not parts:
            return jsonify(error="수동 모드는 교체할 부품을 1개 이상 고르세요"), 400
        no_robot = [valid[p]["name"] for p in parts if not valid[p]["robot_ok"]]
        if no_robot:
            return jsonify(error=f"로봇 교체 불가 부품: {', '.join(no_robot)}"), 400
    # one running repair per robot: the robots (real, simulation) run and stop independently
    if con.execute("SELECT 1 FROM jobs j JOIN vehicles v ON v.id = j.vehicle_id WHERE j.status = 'running' "
                   "AND COALESCE(v.robot_namespace, 'dsr01') = (SELECT COALESCE(robot_namespace, 'dsr01') "
                   "FROM vehicles WHERE id = ?)", (d["vehicle_id"],)).fetchone():
        return jsonify(error="이 로봇에서 이미 진행 중인 수리가 있습니다"), 409
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
    notify_owner(con, jid, "start")
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
    """Jobs in progress. Users only see their own vehicles'. ?vehicle_id= selects a vehicle, ?robot= a robot
    (robot_1, ...; dsr01 for an unassigned vehicle)"""
    where, args = ["j.status = 'running'"], []
    if request.args.get("robot"):
        where.append("COALESCE(v.robot_namespace, 'dsr01') = ?"); args.append(request.args["robot"])
    if role() == "user":
        where.append("v.owner_id = ?"); args.append(session["user_id"])
    if request.args.get("vehicle_id"):
        where.append("j.vehicle_id = ?"); args.append(int(request.args["vehicle_id"]))
    j = db.get_db().execute(JOB_SQL + " WHERE " + " AND ".join(where) + " ORDER BY j.id DESC LIMIT 1", args).fetchone()
    return jsonify(job_detail(j["id"]) if j else None)


@app.get("/api/jobs/<int:job_id>/cloud")
@require()
def job_cloud(job_id):
    """Live measured points of the job in progress (owner page 3D): vehicle coordinates in mm (car.glb axes), with the real vehicle size.
    Own vehicles' jobs only, only while in progress (204 for finished jobs). 304 when unchanged"""
    j = job_or_404(job_id)
    if j["status"] != "running":
        return "", 204
    from datetime import datetime
    started = datetime.strptime(j["started_at"], "%Y-%m-%d %H:%M:%S").timestamp()
    try:
        cloud = robot_files.vehicle_cloud(started, j["robot_namespace"] or "dsr01")
    except (OSError, KeyError, ValueError) as error:
        return jsonify(error=f"측정점을 읽을 수 없습니다: {error}"), 500
    if cloud is None:
        return jsonify(points=[], size=None)
    points, size, mtime = cloud
    tag = f'"{job_id}-{mtime:.3f}"'
    if request.headers.get("If-None-Match") == tag:
        return "", 304
    response = jsonify(points=points, size=size)
    response.headers["ETag"] = tag
    response.headers["Cache-Control"] = "no-cache"
    return response


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
        con.executemany("INSERT INTO findings (job_id, part_code, x, y, z, depth, decision) VALUES (?, ?, ?, ?, ?, ?, ?)",
                        [(job_id, f["part_code"], f.get("x"), f.get("y"), f.get("z"), f.get("depth"), f.get("decision", "dent"))
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
        return jsonify(job_detail(job_id))   # an already finished job stays as is (when the stop button and the robot end overlap)
    con = db.get_db()
    # the status = 'running' condition in the UPDATE changes it only once even if two requests arrive almost together (one mail too)
    changed = con.execute("UPDATE jobs SET status = ?, summary = ?, finished_at = datetime('now','localtime') "
                          "WHERE id = ? AND status = 'running'", (status, d.get("summary"), job_id)).rowcount
    if not changed:
        return jsonify(job_detail(job_id))
    con.execute("INSERT INTO events (job_id, stage, level, message) VALUES (?, 'finish', ?, ?)",
                (job_id, "info" if status in ("done", "no_damage") else "warn", d.get("summary") or status))
    con.commit()
    notify_owner(con, job_id, status)
    return jsonify(job_detail(job_id))


# ================================================================ alarms (admin)
@app.get("/api/alarms")
@require("admin")
def list_alarms():
    """Only warn/error events, newest first. With job number and vehicle name"""
    try:
        limit = max(1, min(int(request.args.get("limit", 100)), 500))
    except ValueError:
        return jsonify(error="limit 은 정수"), 400
    return jsonify(db.rows(db.get_db().execute(
        """SELECT e.id, e.job_id, e.ts, e.stage, e.level, e.message, j.vehicle_id, v.name AS vehicle_name
           FROM events e JOIN jobs j ON j.id = e.job_id JOIN vehicles v ON v.id = j.vehicle_id
           WHERE e.level IN ('warn', 'error') ORDER BY e.id DESC LIMIT ?""", (limit,))))


if __name__ == "__main__":
    db.init_db()
    app.run(host=os.environ.get("HOST", "0.0.0.0"), port=int(os.environ.get("PORT", 5050)), debug=False, threaded=True)
