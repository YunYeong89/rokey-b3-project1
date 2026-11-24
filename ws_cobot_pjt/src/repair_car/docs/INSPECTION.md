# 부품 접촉 검사 (`real_scan`)

`ros2 run repair_car real_scan --part {bumper,bonnet,body,door} [--side left|right] [--controller mesh-height|default] [--plan-only]`

탐침을 잡은 상태(`pick_probe` 이후)에서 차량 부품 하나를 힘 제어로 접촉 주행하며 점군을 측정하고, ROI(손상 후보·확정 영역)를 만듭니다.

## 컨트롤러

| 컨트롤러 | 동작 | 비고 |
|---|---|---|
| `mesh-height` | 메시 높이를 따르는 고정축 래스터. 실기는 DSR `movesx` + 힘·순응 제어, 가상은 Cartesian 스트림 | 웹 부품 검사·자동 수리가 쓰는 경로 |
| `default` (CLI 기본값) | 기준 점군 HeightMap + DSR `movel` 고정축 힘 제어. 후보 영역을 정밀 재방문 | `--box` 시편 검사도 지원 |

부품별 누름 축은 `config/robot.yaml`의 `real_scan.axes`와 `scan.mesh_height` 설정을 따릅니다. 보닛은 BASE −Z, 문은 로봇을 향한 면에서 BASE +X로 누릅니다.

## 처리 순서 (mesh-height)

```
real_scan.main
└── scan_part
    ├── ScanScene.load            실기 배치(real_scene.yaml) 또는 가상 세션 장면
    ├── mesh_scan.run_mesh_scan   경로 계획(캐시) → 실행 → ROI → 보고서
    │   ├── part_scan/height_plan · bonnet_plan · mesh_plan     경로
    │   ├── part_scan/height_dsr (실기) · mesh_run (가상)        실행
    │   └── report/mesh_report → mesh_figures → mesh_roi        ROI·그림
    └── rescan (조건부)           crack ROI 재검출
```

결과는 `InspectionArchive.directory()`가 정한 `data/<이름_번호판_소유자>/<YYYY-MM-DD-HH-MM>/<부품>/`에 저장됩니다 (웹 실행 시 `data/robot_N/data/...`). 파일: `*_plan.json` · `*_route.json` · `*_roi.json` · `*_result.json` · `*_report.png` · `*_roi_reference.ply`. 실행 중에는 `/scan/samples` · `/scan/roi` · `/scan/progress` · `/scan/result`를 발행합니다.

기준 점군 캐시 `pointcloud/real_<part>_reference.ply`는 없으면 다시 만듭니다. `pointcloud/` 폴더 자체는 지우면 안 됩니다.

## ROI 재검출 (rescan)

1차 검사에서 나온 **crack** ROI(`robot.yaml` `rescan.types: [crack]`)를 가까운 것끼리 합쳐 다시 검사합니다. 보닛은 BASE XY, 문은 BASE YZ 평면에서 영역을 만듭니다.

재검출 조건 (`real_scan.py` `scan_part`):
- `--controller mesh-height`, 부품이 보닛 또는 문
- `--plan-only`·`--roi-base`가 없음
- 보닛 편집기 영역(`config/bonnet_path.json`)을 쓰면 `--rescan-drawn-bonnet`이 있어야 함

| 실행 경로 | 보닛 | 문 |
|---|---|---|
| 웹 자동 수리 (`repair_manager`) | 재검출 함 (실기: `--rescan-drawn-bonnet`) | 재검출 함 (가상 demo_final) |
| 웹 부품 검사, 영역 지정 (`--roi-local`) | 재검출 함 | 재검출 함 |
| 웹 부품 검사, 영역 없음 | **실기에서 안 함** (저장된 편집기 영역 사용) | 재검출 함 |
| 터미널 기본 실행 (`--controller default`) | 안 함 | 안 함 |

실기 재검출 실측 기록은 보닛만 있습니다 (2026-11-21, 1차 070610 → 재측정 072515). 문 재검출은 코드에 있으나 실기 실측 기록이 없습니다.

## 관련 프로그램

| 명령 | 용도 |
|---|---|
| `pick_probe` · `release_probe` | 거치대에서 탐침 파지·반납 |
| `choose_car` · `scan_surface` | 모형 차량(sedan/suv/sports) 식별·표면 문지르기 검사 |
| `calibration` · `force_offset` · `motion_offset` · `motion_offset_check` | 힘 보정 (터미널 전용, 결과 그래프는 `images/`) |
| `draw_graph` | 실시간 힘 그래프 (웹은 `--no-png --no-save`로 스트림만) |
| `pointcloud_damage` | 저장된 스캔 비교 (오프라인) |
