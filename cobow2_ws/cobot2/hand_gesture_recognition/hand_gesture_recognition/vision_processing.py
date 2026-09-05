import time
import threading
from collections import Counter, deque

import cv2
import mediapipe as mp

BaseOptions = mp.tasks.BaseOptions
GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
RunningMode = mp.tasks.vision.RunningMode


class CommandRecognizer:
    """test_hand_gesture_recognition.py 에서 검증한 MediaPipe GestureRecognizer를 사용합니다.

    테스트 스크립트는 LIVE_STREAM(비동기 콜백) 모드였지만, 여기서는 VIDEO(동기 호출)
    모드를 사용합니다. 카메라 프레임 타이밍은 이미 CamController가 관리하고 있어서,
    프레임을 넣으면 그 자리에서 바로 결과를 받는 동기 방식이 우리 파이프라인 구조와
    더 잘 맞습니다.
    """

    MIN_SCORE = 0.5  # 이 점수 미만이면 "명령 없음(none)"으로 취급

    def __init__(self, model_path):
        self.model_path = model_path
        self.model = self._load_model(model_path)
        self._last_timestamp_ms = 0

    def _load_model(self, model_path):
        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.VIDEO,
        )
        return GestureRecognizer.create_from_options(options)

    def preprocess(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    def postprocess(self, result):
        if not result.gestures:
            return "none"
        gestures = result.gestures[0]
        if not gestures:
            return "none"
        top = gestures[0]
        if top.score < self.MIN_SCORE:
            return "none"
        return top.category_name

    def predict_command(self, frame):
        """단일 프레임 -> 제스처 라벨.

        VIDEO 모드는 호출마다 타임스탬프가 단조 증가해야 하므로, 실제 시각(ms) 기준으로
        타임스탬프를 넘기되 이전 값보다 반드시 커지도록 보정합니다.
        """
        mp_image = self.preprocess(frame)
        now_ms = int(time.time() * 1000)
        if now_ms <= self._last_timestamp_ms:
            now_ms = self._last_timestamp_ms + 1
        self._last_timestamp_ms = now_ms

        result = self.model.recognize_for_video(mp_image, now_ms)
        return self.postprocess(result)

    def close(self):
        self.model.close()


class SlidingCommandRecognizer:
    """프레임마다 추론은 딱 1번만 하고, 그 결과 라벨을 최근 window_seconds 만큼 버퍼에
    쌓아 다수결로 최종 커맨드를 결정합니다. slide_frames 프레임마다 다수결 결과를
    갱신합니다.

    (윈도우 전체를 매 프레임마다 다시 추론하면 연산량이 window 길이만큼 배가되므로,
    "추론은 프레임당 1회, 다수결은 라벨 버퍼에 대해서만" 하는 방식으로 최적화했습니다.)
    """

    def __init__(self, recognizer: CommandRecognizer, window_seconds: float = 2.0,
                 slide_frames: int = 1, fps: int = 30):
        self.recognizer = recognizer
        window_len = max(1, int(round(window_seconds * fps)))
        self.slide_frames = max(1, slide_frames)

        self._labels = deque(maxlen=window_len)
        self._frame_count = 0
        self._lock = threading.Lock()
        self._latest_command = None
        self._latest_error = None

    def process_frame(self, frame):
        """카메라의 on_frame 콜백에서 새 프레임이 들어올 때마다 호출합니다."""
        try:
            label = self.recognizer.predict_command(frame)
        except Exception as e:
            with self._lock:
                self._latest_error = f"{type(e).__name__}: {e}"
            return

        with self._lock:
            self._labels.append(label)
            self._frame_count += 1
            window_full = len(self._labels) == self._labels.maxlen
            if window_full and self._frame_count % self.slide_frames == 0:
                most_common, _ = Counter(self._labels).most_common(1)[0]
                self._latest_command = most_common
                self._latest_error = None

    def get_latest(self):
        """(command, error) 튜플. 아직 윈도우가 안 찼으면 command는 None."""
        with self._lock:
            return self._latest_command, self._latest_error

    def get_status(self):
        """화면 표시용 상세 상태. command/error 외에 버퍼 채워진 정도도 포함."""
        with self._lock:
            return {
                "command": self._latest_command,
                "error": self._latest_error,
                "buffer_len": len(self._labels),
                "buffer_maxlen": self._labels.maxlen,
                "frame_count": self._frame_count,
            }
