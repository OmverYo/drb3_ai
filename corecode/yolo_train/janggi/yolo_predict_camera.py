from ultralytics import YOLO
import cv2
from pathlib import Path


# ============================================================
# 1. Fruits 디렉터리를 프로젝트 최상위 디렉터리로 설정
# ============================================================
# yolo_predict.py가 있는 디렉터리를 기준으로 사용합니다.
# 따라서 실행 위치가 어디든 Fruits 디렉터리를 정확하게 찾습니다.
BASE_DIR = Path(__file__).resolve().parent


# ============================================================
# 2. 학습된 모델 경로
# ============================================================
# 사용자가 직접 Fruits 디렉터리에 복사한 best.pt
MODEL_PATH = BASE_DIR / "best.pt"

# ============================================================
# 3. best.pt 존재 여부 확인
# ============================================================
if not MODEL_PATH.exists():
    print("오류: best.pt를 찾을 수 없습니다.")
    print(f"확인 경로: {MODEL_PATH}")
    print()
    print("학습 결과의 best.pt를 다음 위치에 복사하세요.")
    print(f"  {BASE_DIR}/best.pt")
    exit()

# ============================================================
# 4. 현재 설정 출력
# ============================================================
print("=" * 60)
print("YOLO Real-Time Prediction")
print("=" * 60)
print(f"Fruits 디렉터리 : {BASE_DIR}")
print(f"Model           : {MODEL_PATH}")
print("=" * 60)

# ============================================================
# 5. YOLO 모델 로드
# ============================================================
model = YOLO(str(MODEL_PATH))

# ============================================================
# 6. 카메라 열기
# ============================================================
# 현재 사용 중인 카메라 번호
CAMERA_ID = 6

cap = cv2.VideoCapture(CAMERA_ID)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print(f"카메라를 열 수 없습니다. Camera ID: {CAMERA_ID}")
    exit()

print()
print("실시간 예측 시작")
print("종료하려면 Q 키를 누르세요.")
print()

# ============================================================
# 7. 실시간 객체 검출
# ============================================================
while True:

    ret, frame = cap.read()

    if not ret:
        print("카메라 프레임을 읽을 수 없습니다.")
        break

    results = model.predict(
        source=frame,
        conf=0.6,
        verbose=False
    )

    # --------------------------------------------------------
    # Prediction 결과
    # --------------------------------------------------------
    result = results[0]

    boxes = result.boxes
    classes = result.names

    for box in boxes:

        # Class ID
        cls_id = int(box.cls[0])

        # Confidence
        conf = float(box.conf[0]) * 100

        # Class 이름 + Confidence
        label = f"{classes[cls_id]} {conf:.1f}%"


        # Bounding Box 좌표
        x1, y1, x2, y2 = map(
            int,
            box.xyxy[0]
        )

        # Bounding Box 그리기
        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )

        # Class 이름 + Confidence 표시
        cv2.putText(
            frame,
            label,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    cv2.imshow(
        "YOLO Predict",
        frame
    )

    # --------------------------------------------------------
    # Q 키 → 종료
    # --------------------------------------------------------
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# ============================================================
# 8. 종료 처리
# ============================================================
cap.release()
cv2.destroyAllWindows()

print("예측 종료")
