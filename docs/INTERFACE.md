# 인터페이스 약속서 (웹 ↔ 로봇)

> 버전 v0.1 · 2026-11-13 · 작성: 웹·DB 담당
> 이 문서를 바꾸면 **팀 전체에 알리고** 맨 아래 변경 이력에 적기. `(협의)` 표시는 아직 확정 전.

## 1. 전체 구조

```
[태블릿 웹] ──rosbridge(ws:9090)──▶ [ROS2] ──▶ 공정 관리 노드 ──▶ 스캔·교체 노드 ──▶ M0609
     ▲                                              │
     └──────── Flask REST API (http:5000) ◀─────────┘  (단계·판정·결과 기록)
```

| 경로 | 무엇을 | 방식 |
|---|---|---|
| 웹 → 로봇 | 시작·정지 명령 | ROS 토픽 `/repair/command` |
| 로봇 → 웹 | 실시간 값 (측정 점, 힘, 단차) | ROS 토픽 `/repair/status` |
| 로봇 → DB | 단계, 판정, 교체 결과, 종료 | REST API (`RepairDB` 클라이언트) |

**주소** (실습 때 채우기)

| 항목 | 값 |
|---|---|
| 웹 서버 (Flask) | `http://<웹 PC IP>:5000` |
| rosbridge | `ws://<로봇 PC IP>:9090` |
| 로봇 API 키 | 헤더 `X-API-Key: rokey-robot-key` (환경변수 `REPAIR_API_KEY`) |

## 2. 공통 약속

**차량 좌표계** — 3D 모델, 스캔, 웹 표시 모두 이 기준

| 항목 | 값 |
|---|---|
| 원점 | 차 뒤쪽 · 왼쪽 모서리 · 바닥 `(협의)` |
| x | 뒤 → 앞 (+), 0 ~ 260 mm `(실측 후 수정)` |
| y | 왼쪽 → 오른쪽 (+), 0 ~ 110 mm `(실측 후 수정)` |
| z | 위 (+) |
| 단위 | mm, N |

**부품 코드**: `bonnet`(보닛), `mirror_l`(사이드미러 좌), `mirror_r`(사이드미러 우)

**단계 이름 (stage)**: `calibrate` → `scan` → `detect` → `stock` → `replace` → `inspect` → `done`
수동 모드는 `scan`, `detect` 생략

**판정 (decision)**: `dent` 찌그러짐 · `missing` 부품 없음 · `suspect` 의심 · `requested` 수동 지정(웹이 기록)

**종료 상태 (status)**: `done` 완료 · `no_damage` 손상 없음 · `on_hold` 재고 없어 보류 · `failed` 실패 · `stopped` 정지

## 3. ROS 토픽

모두 `std_msgs/String`, `data` 안에 JSON 문자열.

### `/repair/command` (웹 → 로봇)

```json
{"cmd": "start", "job_id": 3, "vehicle_id": 1, "mode": "auto", "parts": []}
{"cmd": "start", "job_id": 4, "vehicle_id": 1, "mode": "manual", "parts": ["bonnet", "mirror_r"]}
{"cmd": "stop",  "job_id": 4}
```
- `job_id`는 웹이 DB에 작업을 만든 뒤 보낸다. 로봇은 이 번호로 기록한다.
- `stop`을 받으면 **로봇 동작을 즉시 정지**하고 더 기록하지 않는다 (웹이 `stopped`로 기록함).

### `/repair/status` (로봇 → 웹)

| type | 필드 | 보내는 때 | 예 |
|---|---|---|---|
| `point` | `x, y` [mm], `dz` [mm], `state`, `fine`(선택) | 측정 점 1개 찍을 때마다 | `{"type":"point","x":120,"y":40,"dz":-0.8,"state":"dent"}` |
| `wrench` | `fz` [N] | 10 Hz 이하 | `{"type":"wrench","fz":-4.2}` |
| `flush` | `mm` | 단차 측정 직후 | `{"type":"flush","mm":0.18}` |

- `dz` = 측정 높이 − 정상 높이. **들어가면 음수**.
- `state` = `ok` / `suspect` / `dent` (임계값 기준, 아래 6장).
- `fine: true` = 의심 구역 상세 스캔 점 (화면에 작게 표시).

## 4. REST API (로봇이 쓰는 것)

`RepairDB` 클라이언트 사용 (`car_repair/repair_db_client.py`). 직접 호출 시 헤더 `X-API-Key` 필수.

| 클라이언트 함수 | API | 설명 |
|---|---|---|
| `db.job(job_id)` | `GET /api/jobs/<id>` | 작업 정보 |
| `db.requested_parts(job_id)` | (위와 같음) | 수동 모드 지정 부품 목록 |
| `db.event(job_id, stage, msg, level)` | `POST /api/jobs/<id>/events` | 진행 기록. level = `info`/`warn`/`error` |
| `db.findings(job_id, [...])` | `POST /api/jobs/<id>/findings` | 손상 판정. `[{part_code, x, y, depth, decision}]` |
| `db.part_available(code)` | `GET /api/parts/<code>/check` | 재고 있으면 `True` |
| `db.replacement(job_id, code, success, retries, peak_force, flush_mm)` | `POST /api/jobs/<id>/replacements` | 교체·재검사 결과 |
| `db.consume(code)` | `POST /api/parts/<code>/consume` | 교체 **성공 후** 재고 1개 차감 (0이면 에러 409) |
| `db.finish(job_id, status, summary)` | `POST /api/jobs/<id>/finish` | 작업 종료 (한 번만) |

응답 코드: `200/201` 성공 · `400` 잘못된 값 · `401/403` 키 없음·권한 없음 · `404` 없는 작업 · `409` 재고 없음 / 이미 진행 중

## 5. 로봇 노드 처리 순서

**자동 모드**
```
event(calibrate) → [3점 터치]
event(scan)      → [격자 스캔, 점마다 /repair/status point]
event(detect)    → findings([...])
   손상 없음 → finish("no_damage")
부품마다: part_available() → 없으면 finish("on_hold", "재고 없음: ...")
부품마다: event(replace) → [교체] → event(inspect) → [단차 측정, /repair/status flush]
          → replacement(...) → 성공이면 consume()
event(done) → finish("done") 또는 finish("failed")
```

**수동 모드**: `requested_parts()`로 부품 목록을 읽고 `calibrate` 다음 바로 재고 확인부터 같은 순서.

**에러**: 예외가 나면 `event(stage, 내용, "error")` → `finish("failed", 내용)`. 로봇은 안전 위치로.

## 6. 기준값 `(실험 후 확정)`

| 항목 | 값 | 근거 |
|---|---|---|
| 높이 노이즈 σ | `(측정 예정)` mm | 반복 접촉 20회 실험 |
| 의심 / 손상 임계값 | 1.5σ×3 / 3σ×3 `(임시)` | 찌그러짐 시편 실험 |
| 단차 합격 기준 | 0.5 mm 이하 `(임시)` | |
| 접촉 판정 힘 | `(측정 예정)` N | 힘 노이즈 실험 |

## 7. 예정 (아직 없음)

- 측정 점 DB 저장 API (`POST /api/jobs/<id>/points`) — 고객 3D·이력에서 점 다시 보기용

## 변경 이력

| 날짜 | 버전 | 내용 | 작성 |
|---|---|---|---|
| 2026-11-13 | v0.1 | 최초 작성 | 웹·DB |
