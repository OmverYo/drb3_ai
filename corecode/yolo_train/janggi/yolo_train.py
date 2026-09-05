from ultralytics import YOLO
from datetime import datetime
from pathlib import Path

# ============================================================
# 1. Fruits 디렉터리를 프로젝트 최상위 디렉터리로 설정
# ============================================================
# yolo_train.py가 어느 위치에서 실행되더라도
# yolo_train.py가 들어 있는 Fruits 디렉터리를 기준으로 사용
BASE_DIR = Path(__file__).resolve().parent

# ============================================================
# 2. 기본 파일 설정
# ============================================================
MODEL_FILENAME = "yolov8n.pt"
DATA_YAML_FILENAME = "data.yaml"

model_path = BASE_DIR / MODEL_FILENAME
data_yaml_path = BASE_DIR / DATA_YAML_FILENAME

# ============================================================
# 3. 필수 파일 확인
# ============================================================
if not data_yaml_path.exists():
    raise FileNotFoundError(
        f"data.yaml을 찾을 수 없습니다.\n"
        f"확인할 경로: {data_yaml_path}"
    )

# yolov8n.pt가 없으면 Ultralytics가 자동으로 다운로드하도록 함
# 따라서 여기서는 존재 여부를 강제하지 않음.

# ============================================================
# 4. Train / Validation 결과 디렉터리 이름 생성
# ============================================================
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

train_name = f"train_{timestamp}"
val_name = f"val_{timestamp}"

train_output_dir = BASE_DIR / train_name
val_output_dir = BASE_DIR / val_name


# ============================================================
# 5. 현재 설정 출력
# ============================================================
print("=" * 60)
print("YOLO Training Configuration")
print("=" * 60)

print(f"Fruits 최상위 디렉터리 : {BASE_DIR}")
print(f"Model                  : {model_path}")
print(f"Dataset YAML           : {data_yaml_path}")
print(f"Train 결과             : {train_output_dir}")
print(f"Validation 결과        : {val_output_dir}")

print("=" * 60)

# ============================================================
# 6. YOLO Model 로드
# ============================================================
model = YOLO(str(model_path))

# ============================================================
# 7. YOLO Training
# ============================================================
print("\n[1] YOLO Training 시작\n")

model.train(
    data=str(data_yaml_path),

    epochs=100,
    imgsz=640,
    batch=8,
    patience=20,

    # 결과를 Fruits 아래에 저장
    project=str(BASE_DIR),
    name=train_name
)

# ============================================================
# 8. Validation
# ============================================================
print("\n[2] Validation 시작\n")

model.val(
    data=str(data_yaml_path),

    # 결과를 Fruits 아래에 저장
    project=str(BASE_DIR),
    name=val_name
)

# ============================================================
# 9. 최종 결과 출력
# ============================================================
print("\n" + "=" * 60)
print("YOLO Training 완료")
print("=" * 60)

print(f"Fruits 디렉터리 : {BASE_DIR}")
print(f"Train 결과      : {train_output_dir}")
print(f"Validation 결과 : {val_output_dir}")

print("=" * 60)

