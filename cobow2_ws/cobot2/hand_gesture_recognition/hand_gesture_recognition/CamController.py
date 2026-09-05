from dataclasses import dataclass
import threading
import time

import cv2


@dataclass
class CamConfig:
    device_index: int = 0
    width: int = 640
    height: int = 480
    fps: int = 30


class CamController:
    """카메라를 한 번 열어(open_stream) 노드가 살아있는 동안 계속 스트리밍합니다.

    프레임이 들어올 때마다 on_frame(frame) 콜백을 호출하기만 합니다.
    윈도우/다수결 같은 시간축 후처리는 여기서 하지 않고 vision_model.py의
    SlidingCommandRecognizer가 담당합니다 (관심사 분리: 카메라 IO vs 모델 후처리).
    """

    def __init__(self, config: CamConfig = CamConfig(), on_frame=None):
        self.config = config
        self.cap = None
        self.on_frame = on_frame  # callback(frame: np.ndarray)

        self._thread = None
        self._running = False
        self._last_frame = None
        self._frame_lock = threading.Lock()

    def open_stream(self):
        self.cap = cv2.VideoCapture(self.config.device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.config.fps)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 장치를 열 수 없습니다: index={self.config.device_index}")

    def start_streaming(self):
        """카메라 on 시점부터 계속 프레임을 읽는 백그라운드 스레드를 시작합니다."""
        if self.cap is None:
            raise RuntimeError("open_stream()을 먼저 호출하세요.")
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()

    def _stream_loop(self):
        interval = 1.0 / self.config.fps if self.config.fps > 0 else 0
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                if interval:
                    time.sleep(interval)
                continue

            with self._frame_lock:
                self._last_frame = frame

            if self.on_frame:
                self.on_frame(frame)

            if interval:
                time.sleep(interval)

    def stop_streaming(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def close_stream(self):
        print("stop capturing")
        self.stop_streaming()
        if self.cap:
            self.cap.release()
            self.cap = None

    def get_last_frame(self):
        """가장 최근에 캡처된 프레임을 반환합니다 (없으면 None). 화면 표시용으로 사용."""
        return self._get_last_frame()

    def save_frame(self, filename, frame=None):
        target = frame if frame is not None else self._get_last_frame()
        if target is None:
            raise RuntimeError("저장할 프레임이 없습니다.")
        cv2.imwrite(filename, target)
        print("이미지 저장 완료!")

    def get_jpeg_bytes(self, frame=None):
        target = frame if frame is not None else self._get_last_frame()
        if target is None:
            raise RuntimeError("인코딩할 프레임이 없습니다.")
        ok, buf = cv2.imencode(".jpg", target)
        if not ok:
            raise RuntimeError("JPEG 인코딩 실패")
        return buf.tobytes()

    def _get_last_frame(self):
        with self._frame_lock:
            return self._last_frame
