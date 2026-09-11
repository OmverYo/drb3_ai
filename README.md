# ♟️ 장기 로봇: 음성·손동작 기반 협동로봇 프로토타입

> **RGB-D 기반 객체 위치추정과 좌표변환을 통해 협동로봇 Pick&Place를 수행하고, 음성·손동작 입력을 로봇 명령으로 연결하는 ROS2 기반 멀티모달 프로토타입입니다.**
> 아래 "구현 현황"은 발표 직전(9/11) 기준이며, **완료/부분구현/미구현을 의도적으로 분리**해 표기했습니다. 데모 시연이 가능한 범위와, 실제로 신뢰할 수 없는 범위를 혼동하지 않도록 하는 것이 이 문서의 목적입니다.

---

## 📌 프로젝트 개요

| 항목 | 내용 |
|---|---|
| 🎯 **목표** | 음성 또는 손동작으로 로봇에 명령을 내리고, RGB-D 카메라로 장기판/장기말 위치를 인식해 협동로봇(Doosan M0609+RG2)이 실제 장기말을 집어 이동시키는 시스템 구현 |
| ⚙️ **주요 기능** | 장기말 객체 검출(YOLOv11m) · 좌표 변환(ArUco Hand-Eye Calibration) · 그리퍼 회전 정렬(SAM2 세그멘테이션) · 음성 명령(Whisper+GPT-4o) · 손동작 명령(MediaPipe) · 웹 대시보드(Flask+MongoDB+SocketIO) |
| 🦾 **사용 장비** | Doosan Robotics **M0609**(OnRobot RG2 그리퍼) · Intel RealSense RGB-D 카메라 · 웹캠(제스처 인식용, 로봇 카메라와 별도) · 마이크(음성 입력) · 시판 장기판/장기말 |
| 💻 **개발 환경** | ROS2 Jazzy · Flask · MongoDB Atlas |
| 🛠️ **기술 스택** | ROS2 Service(Trigger, 커스텀 srv) · YOLOv11m · ArUco · SAM2.1 · MediaPipe · Whisper + GPT-4o(LangChain) · Flask REST + SocketIO · MongoDB |
| 📅 **기간** | 2026.08.31 ~ 2026.09.11 |

---

## 🎬 시연 영상

> 🔗 [발표 시연 영상 링크 삽입 필요]

---

## 🏗️ 시스템 아키텍처

```
센싱(RealSense / 웹캠 / 마이크)
        ↓
인식·판단(YOLOv11m / SAM2.1 / MediaPipe 손동작 / Whisper+GPT-4o)
        ↓
좌표 변환(ArUco Hand-Eye Calibration → T_gripper2camera.npy)
        ↓
제어 실행(단일 로봇 제어 노드, 음성/손동작 모드 분기)
```

- ROS2 내부 통신은 Service(Trigger, SrvDepthPosition 등) 중심이며, 로봇 제어부 ↔ 웹 대시보드는 REST + SocketIO로 완전히 분리된 채널을 사용합니다.
- **음성 입력과 손동작 입력은 각각 독립된 경로로 로봇 제어까지 연결되어 있으며, 두 입력을 하나로 중재하는 Command Arbiter(통합 제어 로직)는 설계만 존재하고 구현되지 않았습니다.** 현재는 사용자가 매 순간 둘 중 한 모드를 선택해 사용하는 구조입니다.

---

## 📖 상세 설명

### ❗ 문제정의

- 기존 협동로봇 제어는 티칭펜던트 등 물리적 인터페이스에 의존해, 손동작이 자유로운 사용자와 손동작이 어려운 사용자를 동시에 포괄하지 못함
- 청각장애인의 84% 이상이 '말'로 의사소통하고 수어 사용은 3% 미만이라는 조사 결과 등, 접근성 있는 다중 입력 체계에 대한 수요 존재
- 협동로봇 품질검사(Cognex, Techman 등)는 이미 상용화된 영역이라, 접근성(이중 입력)과 실제 물리적 조작(Manipulation)으로 차별화가 필요

### 💡 해결방안

- 음성(Whisper STT + GPT-4o)과 손동작(MediaPipe 커스텀 모델) 두 입력 경로를 분리 설계해, 동일한 로봇 제어 로직을 두 방식 중 하나로 트리거 가능하도록 구현
- RealSense + YOLOv11m으로 장기말을 검출하고, SAM2.1 세그멘테이션 마스크로 실제 윤곽을 추출해 평면 안전각도(0~179°)를 탐색, ArUco 기반 Hand-Eye Calibration으로 카메라 좌표를 로봇 베이스 좌표로 변환
- 손동작 오인식은 슬라이딩 윈도우 다수결(버퍼링) 방식으로 순간 오인식을 필터링해 신뢰성 확보
- Flask + MongoDB + SocketIO 기반 웹 대시보드로 로봇/게임 상태를 실시간 전파

### 🔄 프로젝트 범위 변경 이력

카드 검사(1차 기획, 설계만 존재·미배포) → 공구 Pick-and-Place → 장기말 조작(최종)으로 범위가 변경되었으며, 이 과정에서 음성 명령 프롬프트가 초기 공구명(hammer/screwdriver/wrench) 기준으로 작성된 잔재가 있어 장기말 14종 명칭으로의 전환 작업이 발표 직전까지 진행되었습니다.

### ✨ 주요기능

| 기능 | 설명 |
|---|---|
| 🃏 객체 검출 | YOLOv11medium으로 장기말 14종(초록·빨강 × 차/마/상/사/포/졸/왕) 검출. 1,104장(공개데이터셋 기반)을 약 2,300장까지 증강해 학습. |
| 📐 좌표 변환 | YOLO bbox → SAM2.1 마스크(윤곽) → Depth 결합 → 카메라 좌표 → T_gripper2camera 변환 → 로봇 베이스 좌표. ArUco 마커는 코너에서 30mm 간격을 두어 장기말에 가려지는 문제를 해결 |
| 🔄 그리퍼 회전 정렬 | SAM2.1 마스크 기준 Distance Transform으로 장애물과의 거리를 계산, 0~179° 전수탐색으로 충돌 없는 최적 그리퍼 각도 산출(9/8~9/10 구현, ) |
| 🎙️ 음성 명령 | 웨이크워드 감지 → Whisper STT → GPT-4o(LangChain) 파싱 → "대상/목적지" 자유발화 명령 추출.|
| ✋ 손동작 명령 | MediaPipe 커스텀 모델(HaGRID 데이터셋 기반) → 숫자 5종(1~5) + 수행 3종(grap/release/bucket) + 제어 2종(reset/none) 총 10가지 제스처 인식. 6 이상의 숫자는 서로 다른 숫자를 연속 입력해 10 이하로 합산하는 방식으로 표현. |
| 🦾 로봇 제어 | 단일 로봇 제어 노드(robot_control.py)가 음성/손동작 모드로 분기해 movel 기반 이동, RG2 그리퍼 개폐 수행.현재는 단일 대상에 대한 즉시 pick 흐름만 동작 |
| 🖥️ 웹 대시보드 | Flask + MongoDB + SocketIO. boards 컬렉션에 장기판 상태(10×9 배열) 저장 및 2초 주기로 실시간 전파. moves/sessions 컬렉션은 설계만 존재하며, 대신 events 컬렉션으로 유사한 이력 로깅을 부분적으로 대체 구현 |

### 🛡️ 안전·보안 — 구현 범위 및 한계 (반드시 확인 필요)

- 손동작 명령 파서는 형식에 맞지 않는 입력을 무시하고 이전 상태를 유지하며 대기(명시적 FSM 클래스는 아니나 상태기계 방식으로 동작)
- 비전 서비스 중복 요청 방지 플래그, 폴링 타임아웃 처리는 구현됨
- **API 보안 미비로 실시간 전파. moves/sessions 컬렉션은 설계만 존재하며, 대신 events 컬렉션으로 유사한 이력 로깅을 부분적으로 대체 구현**: 현재 `PUT /api/board` 등 REST 엔드포인트 대부분이 인증 없이 열려 있고, Flask가 `debug=True`로 구동되며 CORS가 전체 허용 상태입니다. 데모/로컬 환경 밖에서는 반드시 인증 계층 추가가 필요합니다.
- **산업 안전 인증 수준의 Safety Layer(Safety PLC, Emergency Stop, Protective Stop, Safe Zone)는 구현 범위에 포함되지 않습니다.** 실제 사람과 협동 공간을 공유하는 배포를 위해서는 별도의 물리적 안전 계층이 반드시 추가되어야 합니다.

---

## ⚙️ 캘리브레이션 파이프라인로 실시간 전파. moves/sessions 컬렉션은 설계만 존재하며, 대신 events 컬렉션으로 유사한 이력 로깅을 부분적으로 대체 구현

```
TCP 4-point 설정
       ↓
ArUco 마커 부착(코너 4곳, 30mm gap)
       ↓
Camera 촬영 → solvePnP
       ↓
Hand-Eye Calibration
       ↓
T_gripper2camera.npy 생성
       ↓
좌표 변환 검증(reprojection error ≈ 3px)
```

초기에는 수동 티칭(Teaching) 방식을 사용했으나, 장기판 위치가 조금만 움직여도 오차가 발생하고 재보정이 불가능한 구조적 한계가 있어 ArUco 마커 기반 절대좌표 검증 방식으로 전환했습니다.

---

## 🚧 구현 현황

### 완료
- RealSense 데이터 취득, YOLOv11m 장기말 검출(YOLOv8n → YOLOv11m 교체 완료), ArUco Hand-Eye Calibration, M0609+RG2 실제 Pick 동작
- SAM2.1 기반 그리퍼 회전 정렬(정확도 보완은 추후 필요)
- MongoDB(boards 컬렉션) 저장, 웹 대시보드 REST+SocketIO 실시간 반영
- 음성/손동작 각 입력 경로가 개별적으로 로봇 제어까지 연결(단, 통합 중재는 없음)

### 부분구현 / 검증 필요
- YOLOv11m mAP50≈0.995 — 데이터 분리(누수) 방식 재검증 필요
- 게임 이력 저장 — moves/sessions 대신 events 컬렉션으로 기능적 대체
- 음성 명령 대상 명칭 — 공구명 프롬프트 잔재를 장기말 명칭으로 전환 중이었음(전환 완료 여부는 최종 코드 확인 필요)
- API 보안 — 토큰 옵션이 일부 엔드포인트에만 존재, 전면 적용 아님

### 미구현
- Command Arbiter(음성·손동작 통합 중재 로직)
- 장기 게임 상태머신(SELECT_PIECE→HOVER→CONFIRM→PICK_PLACE)
- 합법 수(legal move) 검증 — `janggi_rules.py`/`janggi_move_rules.yaml`로 규칙 엔진 자체는 존재하나, 로봇 제어·웹 API와 아직 연결되지 않음
- 실패 시 안전 정지 조건(Fail-safe)
- 위치오차·Pick 성공률·End-to-End 성공률 등 정량 지표 측정(실측 데이터 없음)

---

## 📦 의존성

- `rclpy`, `std_msgs`
- `dsr_common2`, `dsr_msgs`, 커스텀 인터페이스(Trigger/SrvDepthPosition 등)
- `ultralytics`(YOLOv11), `opencv-python`(ArUco), SAM2.1
- MediaPipe (손동작 인식)
- Whisper, GPT-4o(LangChain) 연동 (음성 인식/파싱)
- `flask`, `flask-socketio`, `pymongo`
- `rg2`(OnRobot RG2 그리퍼 API)

---

## 🔧 주요 트러블슈팅

- 인식 오류: YOLOv8n에서 유사 장기말(예: cha_green ↔ sang_green)을 혼동 → YOLOv11m으로 교체 후 개선
- 그리퍼 파지 실패 → TCP z값 재설정
- Topic → Service 통신 전환(사람이 진행하는 게임 특성상 중간 진행 상태 스트리밍이 불필요해, 요청한 결과만 응답받는 구조로 전환)
- Host-Container 빌드 불일치 → 컨테이너 내부에서 재빌드 절차 확립
- ArUco 마커가 장기말에 가려지는 문제 → 코너에서 30mm 간격(gap) 확보로 해결
- Flask 서버 JSON 전송 시 404 오류(엔드포인트 경로 불일치) 발생 이력 있음
- 카메라가 홈 위치에 있을 때만 장기판 기물 위치를 갱신하도록 제한(그리퍼가 보드 위로 내려가는 동안의 오검출 방지)

---

## 👥 프로젝트 기여자

| 이름 | 연락처 |
|---|---|
| 이동준 | `omver5669@gmail.com` |
| 이정섭 | `jungsub27@gmail.com` |
| 박세준 | `sejun000220@gmail.com` |
| 백승주 | `raybaeksj@gmail.com` |

---

## 🎓 참고자료

- 🔗 [두산로보틱스 튜토리얼](https://robotlab.doosanrobotics.com/ko/Training/OnlineCourses)
- 🔗 [두산로보틱스 M0609 API](https://v2-manual.scroll.site/ko/v2-programming-manual/2.12.1/publish)
- 🔗 HaGRID (Hand Gesture Recognition Image Dataset)
