# 포함된 벤더 코드

이 배포본은 기존 개발 워크스페이스의 수정된 소스를 필요한 범위로 복사했습니다.
원본 워크스페이스에 벤더 Git 이력이 없으므로 upstream commit SHA를 특정하지 않습니다.

| 경로 | 내용 | 포함된 원본 라이선스 |
|---|---|---|
| `ws_cobot_pjt/src/cobot_rg2/doosan-robot2/` | Doosan SDK, ROS 인터페이스·하드웨어·컨트롤러, M0609 메시 | 해당 디렉토리 `LICENSE` (Apache 2.0) |
| `ws_cobot_pjt/src/cobot_rg2/onrobot-ros2/` | OnRobot RG 메시지·Modbus 드라이버, RG2 메시 | 해당 디렉토리 `LICENSE` (MIT) |
| `ws_cobot_pjt/src/rokey/` | 기존 simple_move 코드 | 해당 디렉토리 `LICENSE` (MIT) |

배포용 변경: 테스트·예제·미사용 모델/문서 제외, M0609/RG2 description 패키지의
설치 대상을 메시로 한정, 런타임 manifest 정리, 미사용 Modbus 패키지 의존 이름 정리.
기존 개발본의 Jazzy 대응 SDK·컨트롤러·그리퍼 변경은 유지합니다.

`dsr_common2/lib/jazzy/{arm64,x86_64}/libDRFL.a`는 Doosan SDK의 필수 링크 입력입니다.
사용자 PC에서 생성된 컴파일 산출물과 구분하여 저장소에 포함합니다.
Doosan DRCF 에뮬레이터는 이미지 주소와 digest로 참조하며 이미지 자체는 저장하지 않습니다.
