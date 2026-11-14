# 자동차 자동 수리 로봇

ROKEY 협동-1 · B그룹 3조

두산 M0609가 **촉각(힘 센서)으로 차 표면을 스캔**해 손상 부위를 찾고, 부품(보닛·사이드미러)을 교체한 뒤 단차를 재검사하는 로봇 시스템입니다. 비전 없이 힘 제어만 사용합니다.
차량 위치 확인 → 표면 스캔 → 손상 판정 → 재고 확인 → 부품 교체 → 재검사(단차) → 완료
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
├── web/             # 기존 웹 프로젝트
├── docs/            # 기존 프로젝트 문서
└── ws_cobot_pjt/     # ROS 기본 제어 배포본
    ├── native/      # macOS Gazebo/RViz 표시
    └── ws_cobot_pjt/src/ # 공통 ROS 패키지
```
