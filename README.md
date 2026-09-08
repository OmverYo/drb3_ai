# ♟️ 장기 로봇: 음성·손동작 기반 협동로봇 프로토타입

> **RGB-D 기반 객체 위치추정과 좌표변환을 통해 협동로봇 Pick&Place를 수행하고, 음성·손동작 입력을 로봇 명령으로 연결한 ROS2 기반 멀티모달 프로토타입**

---

## 📌 프로젝트 개요

| 항목 | 내용 |
| ------------- | ----------------------------------------------------------------------------------------------- |
| 🎯 **목표** | 음성 또는 손동작으로 로봇에 명령을 내리고, RGB-D 카메라로 장기판/장기말 위치를 인식해 협동로봇(Doosan M0609+RG2)이 실제 장기말을 집어 이동시키는 시스템 구현 |
| ⚙️ **주요 기능** | 장기말/공구 객체 검출(YOLOv8) · 좌표 변환(ArUco Hand-Eye Calibration) · 음성 명령(Whisper+GPT-4o) · 손동작 명령(MediaPipe) · 웹 대시보드(Flask+MongoDB+SocketIO) |
| 🦾 **사용 장비** | Doosan Robotics **M0609**(OnRobot RG2 그리퍼) · Intel RealSense RGB-D 카메라 · 웹캠(제스처 인식용, 로봇 카메라와 별도) · 마이크(음성 입력) · 체커보드(캘리브레이션 20장) · 시판 장기판/장기말 |
| 💻 **개발 환경** | ROS2 Jazzy · Flask · MongoDB Atlas |
| 🛠️ **기술 스택** | ROS2 Service(Trigger, 커스텀 srv) · YOLOv8 · ArUco · MediaPipe · Whisper + GPT-4o(LangChain) · Flask REST + SocketIO · MongoDB |
| 📅 **기간** | 2026.08.31 ~ 2026.09.11 |

---

## 🎬 시연 영상

> 🔗 [발표 시연 영상 링크 삽입 필요]

---

## 🏗️ 시스템 아키텍처

```
센싱(RealSense / 웹캠 / 마이크)
        ↓
인식·판단(YOLOv8 / MediaPipe 손동작 / Whisper+GPT-4o)
        ↓
좌표 변환(ArUco Hand-Eye Calibration → T_gripper2camera.npy)
        ↓
제어 실행(단일 로봇 제어 노드, 음성/손동작 모드 분기 — 통합 중재 로직은 미구현)
```

ROS2 내부 통신은 Service(Trigger, SrvDepthPosition 등) 중심이며, 로봇 제어부 ↔ 웹 대시보드는 REST + SocketIO로 완전히 분리된 채널을 사용합니다.

---

## 📖 상세 설명

### ❗ 문제정의

* 기존 협동로봇 제어는 티칭펜던트 등 물리적 인터페이스에 의존해, 손동작이 자유로운 사용자와 손동작이 어려운 사용자를 동시에 포괄하지 못함
* 청각장애인의 84% 이상이 '말'로 의사소통하고 수어 사용은 3% 미만이라는 조사 결과 등, 접근성 있는 다중 입력 체계에 대한 수요 존재
* 협동로봇 품질검사(Cognex, Techman 등)는 이미 상용화된 영역이라, 접근성(이중 입력)과 실제 물리적 조작(Manipulation)으로 차별화가 필요

### 💡 해결방안

* 음성(Whisper STT + GPT-4o)과 손동작(MediaPipe 커스텀 모델) 두 입력 경로를 분리 설계해, 동일한 로봇 제어 로직을 두 방식 중 하나로 트리거 가능하도록 구현
* RealSense + YOLOv8로 대상 객체(장기말/공구)를 검출하고, ArUco 기반 Hand-Eye Calibration으로 픽셀 좌표를 로봇 베이스 좌표로 변환
* 손동작 오인식은 슬라이딩 윈도우 다수결 방식으로 순간 오인식을 필터링해 신뢰성 확보
* Flask + MongoDB + SocketIO 기반 웹 대시보드로 로봇/게임 상태를 실시간 전파

### 🔄 프로젝트 범위 변경 이력

카드 검사(1차 기획, 설계만 존재·미배포) → 공구 Pick-and-Place → 장기말 조작(최종)으로 범위가 변경되었으며, 이 과정에서 YOLOv8 객체 클래스가 공구 5종(drill/hammer/pliers/screwdriver/wrench)과 장기말 14종(초록·빨강 × 차/마/상/사/포/졸/왕)으로 이원화되어 남아 있습니다.

### ✨ 주요기능

| 기능 | 설명 |
| --------------- | ------------------------------------------------------------------- |
| 🃏 객체 검출 | YOLOv8로 장기말 14종/공구 5종 검출. 장기말 모델은 1,104장(공개데이터셋)을 Roboflow로 2,300장까지 증강해 100epoch 학습, mAP50≈0.995(검증 데이터 분리 방식 재확인 필요) |
| 📐 좌표 변환 | YOLO bbox 중심 → Depth 값 → 카메라 좌표 → T_gripper2camera 변환 → 로봇 베이스 좌표. ArUco 보드는 지정된 홈 위치에서만 갱신되도록 제한(그리퍼 하강 중 오검출 방지) |
| 🎙️ 음성 명령 | 웨이크워드 감지 → Whisper STT → GPT-4o(LangChain) 파싱 → "대상/목적지" 자유발화 명령 추출 |
| ✋ 손동작 명령 | MediaPipe 커스텀 모델(v2→v3 재학습) → 숫자(0~10)+콤마+grap/release/bucket/reset 상태기계 → 좌표 시퀀스 명령 |
| 🦾 로봇 제어 | 단일 로봇 제어 노드(robot_control.py)가 음성/손동작 모드로 분기해 movel 기반 이동, RG2 그리퍼 개폐 수행. **두 입력을 하나로 중재하는 Command Arbiter는 현재 설계만 존재하며 미구현** |
| 🖥️ 웹 대시보드 | Flask + MongoDB + SocketIO. boards 컬렉션에 장기판 상태(10×9 배열) 저장 및 실시간 전파. **moves/sessions 컬렉션은 설계만 존재, 미구현** |

### 🛡️ 안전 설계 — 구현 범위 및 한계

* 손동작 명령 파서: 형식에 맞지 않는 입력은 무시하고 이전 상태를 유지하며 대기(명시적 FSM 클래스는 아니나 상태기계 방식으로 동작)
* 비전 서비스 중복 요청 방지 플래그, 폴링 타임아웃 처리
* **미구현 — 반드시 확인 필요**: 산업 안전 인증 수준의 Safety Layer(Safety PLC, Emergency Stop, Protective Stop, Safe Zone)는 현재 구현 범위에 포함되지 않음. 실제 사람과 협동 공간을 공유하는 배포를 위해서는 별도 물리적 안전 계층이 반드시 추가되어야 함

---

## ⚙️ 캘리브레이션 파이프라인

```
TCP 4-point 설정
       ↓
Checkerboard 설치
       ↓
Camera 촬영
       ↓
Hand-Eye Calibration
       ↓
T_gripper2camera.npy 생성
       ↓
좌표 변환 검증
```

---

## 🚧 구현 현황

**완료된 것**
* RealSense 데이터 취득, YOLOv8 검출(공구/장기말), Hand-Eye Calibration, M0609+RG2 실제 동작
* 음성/손동작 두 입력 경로 모두 로봇 제어까지 개별 분기 연결(엔드투엔드)
* MongoDB(boards 컬렉션) 저장, 웹 대시보드 실시간 반영
* Command Arbiter(음성·손동작 통합 중재 로직) 실제 구현
* 위치오차·Pick 성공률·End-to-End 성공률 등 정량 지표 측정("동작한다" ≠ "검증됐다" — Detection 성공률과 Pick 성공률은 분리 측정 필요, 현재 둘 다 미측정)
* ArUco 보드 기울기/정렬, 뎁스 약 1cm 오차 안정화

---

## 📦 의존성

* `rclpy`, `std_msgs`
* `dsr_common2`, `dsr_msgs`, `custom_interfaces` (Trigger/SrvDepthPosition 등 커스텀 서비스 정의)
* `ultralytics`(YOLOv8), `opencv-python`(ArUco)
* MediaPipe (손동작 인식)
* Whisper, GPT-4o(LangChain) 연동 (음성 인식/파싱)
* `flask`, `flask-socketio`, `pymongo`
* `rg2`(OnRobot RG2 그리퍼 API)

---

## 🔧 주요 트러블슈팅

* 그리퍼 파지 실패 → TCP z값 재설정
* Topic → Service 통신 전환 (사람이 진행하는 게임 특성상 중간 진행 상태 스트리밍이 불필요해, 요청한 결과만 응답받는 구조로 전환)
* Host-Container 빌드 불일치 → 컨테이너 내부에서 재빌드
* 1920x1080 학습 이미지 잘림 → 전처리 코드 수정
* 객체 위치 계산 시 Y값 오프셋 보정(그리퍼가 더 깊이 들어가 잡을 수 있도록 -5 / -2.5 / 5 / 15 / 12.5 순으로 총 5회 시행착오 후 보정값 확정)
* 서버 IP 하드코딩 문제 — 로봇/비전 시스템을 구동하는 노트북이 바뀔 때마다 IP를 다시 설정해야 하는 제약 존재
* 카메라가 홈 위치에 있을 때만 장기판 기물 위치를 갱신하도록 제한(그리퍼가 보드 위로 내려가는 동안의 오검출 방지)

---

## 👥 프로젝트 기여자

| 이름 | 연락처 |
| --- | ------------------------- |
| 이동준 | `omver5669@gmail.com` |
| 이정섭 | `jungsub27@gmail.com` |
| 박세준 | `sejun000220@gmail.com` |
| 백승주 | `raybaeksj@gmail.com` |

---

---

## 🎓 참고자료

* 🔗 [두산로보틱스 튜토리얼](https://robotlab.doosanrobotics.com/ko/Training/OnlineCourses)
* 🔗 [두산로보틱스 M0609 API](https://v2-manual.scroll.site/ko/v2-programming-manual/2.12.1/publish)
* 🔗 [참고 자료 추가 필요]
