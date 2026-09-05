import cv2
import time
import mediapipe as mp

BaseOptions = mp.tasks.BaseOptions
GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
RunningMode = mp.tasks.vision.RunningMode


# Gesture Recognition 결과 callback
def print_result(result, output_image, timestamp_ms):

    if not result.gestures:
        return

    for gestures in result.gestures:

        if not gestures:
            continue

        gesture = gestures[0]

        print(
            f"Gesture: {gesture.category_name}, "
            f"Score: {gesture.score:.3f}"
        )


options = GestureRecognizerOptions(
    base_options=BaseOptions(
        model_asset_path="src/hand_gesture_recognition/resource/gesture_recognizer.task"
    ),
    running_mode=RunningMode.LIVE_STREAM,
    result_callback=print_result,
)


with GestureRecognizer.create_from_options(options) as recognizer:

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        exit()

    while True:

        ret, frame = cap.read()

        if not ret:
            print("웹캠 프레임을 읽을 수 없습니다.")
            break

        rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb_frame
        )

        timestamp_ms = int(time.time() * 1000)

        recognizer.recognize_async(
            mp_image,
            timestamp_ms
        )

        cv2.imshow(
            "MediaPipe Gesture Recognition",
            frame
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()