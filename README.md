# 자동차 자동 수리 로봇

ROKEY 협동-1 · B그룹 3조

두산 M0609가 **촉각(힘 센서)으로 차 표면을 스캔**해 손상 부위를 찾고, 부품(보닛·사이드미러)을 교체한 뒤 단차를 재검사하는 로봇 시스템입니다. 비전 없이 힘 제어만 사용합니다.
표면 스캔 → 손상 판정 → 재고 확인 → 부품 교체 → 재검사(단차) → 완료
(수동 모드는 스캔·판정을 건너뛰고 지정 부품만 교체)

## 팀원
| 이름 | 담당 |
|---|---|
| 조윤영 | |
| 최원준 | |
| 송세진 | |
| 지민관 | |

## ROS 기본 제어 배포본

M0609 `simple_move`와 OnRobot RG2 그리퍼 제어의 설치·실행 방법은
[ws_cobot_pjt/README.md](ws_cobot_pjt/README.md)에 정리했습니다.
macOS native(Docker ROS + Gazebo/RViz) 실행과 Ubuntu ROS 직접 실행을 구분합니다.
Docker 이미지 파일은 별도 공유하지 않으며, 문서의 Docker 이미지·ROS 워크스페이스
빌드 절차를 따라 각 PC에서 생성합니다.

```text
rokey-b3-project1/
├── web/              # 웹 (Flask, :5050) — 관리자 정비 관제 · 로봇 탭 · 고객 화면 · 알림 메일
├── ws_cobot_pjt/     # ROS 워크스페이스 배포본
│   ├── native/       # macOS Gazebo/RViz 표시, 세션 관리 (session.py)
│   ├── run_web.sh    # 웹 + rosbridge + web_control 함께 실행
│   └── ws_cobot_pjt/src/  # ROS 패키지 (rokey, cobot_rg2, rokey_interfaces …)
├── part_replace/     # 부품 교체 (보닛 분해·조립)
├── cad/              # 3D 모델 (차량, 탐침, 손상 보닛)
└── docs/             # 프로젝트 문서
```

## 웹 실행

```bash
cd web && python3 app.py                 # http://<PC IP>:5050
./ws_cobot_pjt/run_web.sh start          # 로봇 세션에 웹 · rosbridge 붙여서 실행 (로봇 탭)
```

자세한 내용은 [web/README.md](web/README.md) 를 보세요.
