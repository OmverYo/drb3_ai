from ultralytics import YOLO

model = YOLO("yolov8n.pt")

results = model(["sample2.jpg", "sample.jpg"], imgsz=640)


for result in results:
    result.show()

print(results)