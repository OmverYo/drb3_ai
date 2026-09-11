♟️ 장기 로봇: 음성·손동작 기반 협동로봇 프로토타입 (ROS2 + YOLOv11 + MongoDB)
============================================================================

RGB-D 기반 객체 위치추정과 좌표변환을 통해 협동로봇 Pick&Place를 수행하고, 음성·손동작 입력을 로봇 명령으로 연결하는 ROS2 기반 멀티모달 프로토타입입니다.
RealSense로 장기말을 인식(YOLOv11m)하고, ArUco Hand-Eye Calibration으로 좌표를 변환한 뒤, 음성(Whisper+GPT-4o) 또는 손동작(MediaPipe)으로 받은 명령에 따라 Doosan M0609 협동로봇이 실제 장기말을 집어 이동시킵니다.

> 아래 "구현 현황"은 발표 직전(2026.09.11) 기준이며, 완료/부분구현/미구현을 의도적으로 분리해 표기했습니다. 데모 시연이 가능한 범위와 실제로 신뢰할 수 없는 범위를 혼동하지 않도록 하는 것이 이 문서의 목적입니다.

---

📌 프로젝트 개요
----------------

| 항목 | 내용 |
|---|---|
| 🎯 목표 | 음성 또는 손동작으로 로봇에 명령을 내리고, RGB-D 카메라로 장기판/장기말 위치를 인식해 협동로봇(Doosan M0609+RG2)이 실제 장기말을 집어 이동시키는 시스템 구현 |
| ⚙️ 주요 기능 | 장기말 객체 검출(YOLOv11m) · 좌표 변환(ArUco Hand-Eye Calibration) · 그리퍼 회전 정렬(SAM2.1 세그멘테이션) · 음성 명령(Whisper+GPT-4o) · 손동작 명령(MediaPipe) · 웹 대시보드(Flask+MongoDB+SocketIO) |
| 🦾 사용 장비 | Doosan Robotics M0609(OnRobot RG2 그리퍼) · Intel RealSense RGB-D 카메라 · 웹캠(제스처 인식용, 로봇 카메라와 별도) · 마이크(음성 입력) · 시판 장기판/장기말 14종 |
| 💻 개발 환경 | ROS2 Jazzy · Flask · MongoDB Atlas |
| 📅 기간 | 2026.08.31 ~ 2026.09.11 (팀 b-3, ROKEY 부트캠프 9기) |

---

📌 주요 기능 (Key Features)
---------------------------

### 1. 장기말 객체 검출 (YOLOv11m)
- RealSense RGB-D로 장기판을 촬영해 장기말 14종(초록·빨강 × 차/마/상/사/포/졸/왕)을 검출합니다.
- 1,104장(공개 데이터셋 기반)을 약 2,300장까지 증강해 학습했습니다.
- 초기 YOLOv8n 대비 인식 정확도가 크게 개선되어 최종 채택했습니다(특히 초록 상↔차 혼동 사례 개선).
- mAP50≈0.995로 보고되었으나, train/val 데이터 분리(누수) 여부가 재검증되지 않은 상태라 수치를 그대로 신뢰하기는 어렵습니다.

### 2. 좌표 변환 (ArUco Hand-Eye Calibration)
- YOLO bbox → SAM2.1 마스크(윤곽) → Depth 결합 → 카메라 좌표 → `T_gripper2camera.npy` 변환 → 로봇 베이스 좌표 순으로 처리합니다.
- ArUco 마커는 코너 4곳에 부착하며, 장기말에 가려지는 문제는 코너에서 30mm 간격을 두어 해결했습니다.

### 3. 그리퍼 회전 정렬 (SAM2.1 세그멘테이션)
- SAM2.1 마스크 기준 Distance Transform으로 장애물과의 거리를 계산하고, 0~179° 전수탐색으로 충돌 없는 최적 그리퍼 각도를 산출합니다.
- 정확도 보완이 필요한 상태로 발표에 투입되었습니다.

### 4. 음성 명령 (Whisper + GPT-4o)
- 웨이크워드 감지 → Whisper STT → GPT-4o(LangChain) 파싱 → "대상/목적지" 자유발화 명령 추출.
- 예: "빨간왕 오행 우열로 옮겨줘" → `wang_red, target 5,5`
- 음성 신뢰도 임계값은 PRD상 0.60으로 정의되어 있으나, 실측 검증 근거는 확보되지 않았습니다.

### 5. 손동작 명령 (MediaPipe)
- MediaPipe 커스텀 모델(HaGRID 데이터셋 기반) → 숫자 5종(1~5) + 수행 3종(grab/release/bucket) + 제어 2종(reset/none), 총 10가지 제스처를 인식합니다.
- 6 이상의 숫자는 서로 다른 숫자를 연속 입력해 합산하는 방식으로 표현합니다.
- PRD상 손동작 신뢰도 임계값은 0.70이나, 실제 코드 기준값은 0.5로 확인되어 문서와 구현이 불일치합니다(정정 필요).

### 6. 로봇 제어
- 단일 로봇 제어 노드(`robot_control.py`)가 음성/손동작 모드로 분기해 movel 기반 이동, RG2 그리퍼 개폐를 수행합니다.
- 음성·손동작 두 입력을 하나로 중재하는 Command Arbiter, 장기 게임 상태머신(SELECT_PIECE→HOVER→CONFIRM→PICK_PLACE), 합법 수(legal move) 검증은 모두 미구현이며, 현재는 단일 대상에 대한 즉시 pick 흐름만 동작합니다.

### 7. 웹 대시보드
- Flask + MongoDB + SocketIO 기반. `boards` 컬렉션에 장기판 상태(10×9 배열)를 저장하고 2초 주기로 실시간 전파합니다.
- `moves`/`sessions` 컬렉션은 설계만 존재하며, 대신 `events` 컬렉션으로 유사한 이력 로깅을 부분적으로 대체 구현했습니다.

---

📌 시스템 설계 (System Architecture)
------------------------------------

<img width="1751" height="903" alt="Screenshot from 2026-09-11 11-48-23" src="https://github.com/user-attachments/assets/7e613620-c300-49d9-babd-cf510d339233" />
<img width="1751" height="903" alt="Screenshot from 2026-09-11 11-48-35" src="https://github.com/user-attachments/assets/7a641100-4e73-459b-a45c-30f05e017442" />


### 전체 구조

시스템은 크게 센싱(Sensing) → 인식·판단(Perception) → 좌표 변환(Transform) → 제어 실행(Control) 네 단계로 구성됩니다.

```
센싱(RealSense / 웹캠 / 마이크)
        ↓
인식·판단(YOLOv11m / SAM2.1 / MediaPipe 손동작 / Whisper+GPT-4o)
        ↓
좌표 변환(ArUco Hand-Eye Calibration → T_gripper2camera.npy)
        ↓
제어 실행(robot_control 단일 노드, 음성/손동작 모드 분기)
        ↓
웹 대시보드(Flask + MongoDB + SocketIO)로 실시간 전파
```

### ROS2 노드 구성

| 노드 | 패키지 | 역할 |
|---|---|---|
| `get_keyword` | voice_processing | 웨이크워드 감지 → STT → GPT-4o 파싱 → 명령 서비스(`/get_keyword`) 제공 |
| `get_command` | hand_gesture_recognition | 웹캠 손동작 인식 → 문장 단위 명령 조합 → 명령 서비스(`/get_command`) 제공 |
| `object_detection` | object_detection (Docker 컨테이너) | RealSense 영상 → YOLOv11m 검출 → ArUco 좌표계 → 3D 위치 서비스 제공 |
| `robot_control` | robot_control | pick_and_place 허브. 음성/손동작 명령 수신 → 좌표 조회 → M0609 이동 및 RG2 그리퍼 제어 |
| `task_json` | robot_control | 이벤트(`/api/events`) 로깅 노드 |
| `app.py`(Flask) | janggi-web | REST(`/api/board`, `/api/test_move`) + SocketIO로 웹 대시보드에 보드 상태 실시간 전파 |

- ROS2 내부 통신은 Service(Trigger, `SrvDepthPosition`, `SrvGraspPlan` 등) 중심이며, 로봇 제어부 ↔ 웹 대시보드는 REST + SocketIO로 완전히 분리된 채널을 사용합니다.
- Object Detection 노드는 별도 Docker 컨테이너(`yolo-detection`)에서 실행되며, ROS2 서비스 통신으로 `robot_control`과 연결됩니다.

---

📌 개발 환경 (Environment)
---------------------------

- **OS**: Ubuntu 24.04 권장
- **Middleware**: ROS 2 Jazzy
- **Container**: Docker (YOLOv11m 객체 검출 노드 격리 실행)
- **Language**: Python 3.12.3
- **Database**: MongoDB Atlas
- **Web Server**: Flask + Flask-SocketIO
- **Key Libraries**: `rclpy`, `ultralytics`(YOLOv11), `opencv-python`, `mediapipe`, `openai-whisper`, `langchain-openai`, `pymongo`, `flask-socketio`

---

📌 사용 장비 (Hardware Setup)
------------------------------

본 프로젝트는 Doosan Robotics **M0609** 협동로봇을 기준으로 개발되었습니다.

| Component | Type | 비고 |
|---|---|---|
| Robot | Doosan M0609 (6축 협동로봇) | ROS2 dsr_common2/dsr_msgs 연동 |
| Gripper | OnRobot RG2 | 개구부 50mm, 파지력 3N |
| Vision (로봇용) | Intel RealSense (RGB-D) | 장기판/장기말 3D 위치 인식 |
| Vision (제스처용) | 웹캠 (로봇 카메라와 별도) | 손동작 인식 전용 |
| Audio | 마이크 | 웨이크워드 감지 및 STT 입력 |
| 대상물 | 시판 장기판 · 장기말 14종 | 초록·빨강 × 차/마/상/사/포/졸/왕 |
| 캘리브레이션 도구 | ArUco 마커 4매 | 장기판 코너 4곳, 30mm 간격 부착 |

---

📌 의존성 설치 (Installation)
------------------------------

### 1. Python 필수 라이브러리

YOLOv11m 구동, 손동작·음성 인식, 웹 서버 구동을 위한 패키지입니다. (저장소에 `requirements.txt`가 아직 포함되어 있지 않아 아래 목록을 기준으로 별도 생성·관리를 권장합니다.)

```bash
pip install ultralytics opencv-python mediapipe numpy
pip install openai-whisper langchain langchain-openai
pip install flask flask-socketio pymongo
```

### 2. ROS2 패키지 설치

```bash
sudo apt update
sudo apt install ros-jazzy-desktop
# Doosan 협동로봇 전용 패키지 (M0609, RG2)
# dsr_common2, dsr_msgs, onrobot 관련 드라이버는 두산로보틱스 공식 저장소 참고
```

### 3. Docker (YOLO 검출 노드용)

```bash
docker pull <yolo-detection 이미지>
docker create --name yolo-detection <이미지>
```

### 4. MongoDB Atlas

- `db_config.py`에 `MONGODB_URI`를 본인 클러스터 접속 문자열로 설정합니다.
- 컬렉션: `janggi.boards`, `janggi.events` (자동 생성)

---

📌 실행 순서 (How to Run)
--------------------------

전체 시스템을 구동하기 위해 아래 순서대로 터미널을 실행하세요.

#### 1. 로봇/카메라 공통 노드 실행 (`common.launch.py`)

M0609 로봇 연결, RealSense 카메라, 이벤트 로깅 노드를 함께 실행합니다.

```bash
# 터미널 1
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_control common.launch.py
```

#### 2. 제어 노드 실행 (`control.launch.py`)

Docker 기반 YOLO 검출 노드, 로봇 제어 노드, 입력 모드(음성/손동작) 노드를 함께 실행합니다. `mode` 인자로 입력 방식을 선택합니다.

```bash
# 터미널 2 — 손동작(vision) 모드로 실행할 경우
ros2 launch robot_control control.launch.py mode:=vision

# 또는 음성(voice) 모드로 실행할 경우
ros2 launch robot_control control.launch.py mode:=voice
```

> 내부적으로 다음이 함께 실행됩니다.
> - `docker start yolo-detection` → 컨테이너 내부에서 `ros2 run object_detection object_detection`
> - `robot_control` 노드 (`--mode` 인자로 분기)
> - `mode:=voice`일 때 `voice_processing`의 `get_keyword` 노드
> - `mode:=vision`일 때 `hand_gesture_recognition`의 `get_command` 노드

#### 3. 웹 대시보드 실행

```bash
# 터미널 3
cd janggi-web
python3 app.py
# http://localhost:5000 에서 실시간 장기판 상태 확인
```

---

🚧 구현 현황
-------------

### 완료
- RealSense 데이터 취득, YOLOv11m 장기말 검출(YOLOv8n → YOLOv11m 교체 완료), ArUco Hand-Eye Calibration, M0609+RG2 실제 Pick 동작
- SAM2.1 기반 그리퍼 회전 정렬(정확도 보완은 추후 필요)
- MongoDB(boards 컬렉션) 저장, 웹 대시보드 REST+SocketIO 실시간 반영
- 음성/손동작 각 입력 경로가 개별적으로 로봇 제어까지 연결(단, 통합 중재는 없음)

### 부분구현 / 검증 필요
- YOLOv11m mAP50≈0.995 — 데이터 분리(누수) 방식 재검증 필요
- 게임 이력 저장 — moves/sessions 대신 events 컬렉션으로 기능적 대체
- 음성 명령 대상 명칭 — 공구명 프롬프트 잔재를 장기말 명칭으로 전환 중이었음
- API 보안 — 토큰 옵션이 일부 엔드포인트에만 존재, 전면 적용 아님(`debug=True`, CORS 전체 허용 상태)

### 미구현
- Command Arbiter(음성·손동작 통합 중재 로직)
- 장기 게임 상태머신(SELECT_PIECE→HOVER→CONFIRM→PICK_PLACE)
- 합법 수(legal move) 검증 — `janggi_rules.py`/`janggi_move_rules.yaml`로 규칙 엔진 자체는 존재하나, 로봇 제어·웹 API와 아직 연결되지 않음
- 실패 시 안전 정지 조건(Fail-safe)
- 위치오차·Pick 성공률·End-to-End 성공률 등 정량 지표 측정(실측 데이터 없음)

---

🔧 주요 트러블슈팅
-------------------

- 인식 오류: YOLOv8n에서 유사 장기말(cha_green ↔ sang_green)을 혼동 → YOLOv11m 교체 후 개선
- 그리퍼 파지 실패 → TCP z값 재설정
- Topic → Service 통신 전환(중간 진행 상태 스트리밍이 불필요해, 요청한 결과만 응답받는 구조로 전환)
- Host-Container 빌드 불일치 → 컨테이너 내부 재빌드 절차 확립
- ArUco 마커가 장기말에 가려지는 문제 → 코너에서 30mm 간격 확보로 해결
- 카메라가 홈 위치에 있을 때만 장기판 기물 위치를 갱신하도록 제한(그리퍼가 보드 위로 내려가는 동안의 오검출 방지)

---

👥 프로젝트 기여자
-------------------

| 이름 | 담당 | 연락처 |
|---|---|---|
| 이동준 | YOLO 학습, Web·Server·DB, Get Keyword 프롬프트, Git 협업 관리 | `omver5669@gmail.com` |
| 이정섭 | Gesture 모델·통신, Docker 환경 | `jungsub27@gmail.com` |
| 박세준 | ROS2 통신, DB 연결, 음성·손동작 제어, Segmentation | `sejun000220@gmail.com` |
| 백승주 | 기획, 협업 일정 및 자료 관리, 발표 자료 | `raybaeksj@gmail.com` |

---

🎓 참고자료
------------

- [두산로보틱스 튜토리얼](https://robotlab.doosanrobotics.com/ko/Training/OnlineCourses)
- [두산로보틱스 M0609 API](https://v2-manual.scroll.site/ko/v2-programming-manual/2.12.1/publish)
- HaGRID (Hand Gesture Recognition Image Dataset)
