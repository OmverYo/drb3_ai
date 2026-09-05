import cv2


class WebcamViewer:
    """실시간 카메라 프레임과 인식 상태를 창에 그려주는 뷰어.

    get_command.py가 CamController(영상 프레임)와 SlidingCommandRecognizer(인식 상태)로부터
    각각 최신 데이터를 받아와서 이 클래스에 넘겨주면, 이 클래스는 "그리기/표시"만 담당합니다.
    (get_command.py가 vision_processing.py의 결과를 받아와 쓰는 것과 동일한 패턴입니다.)
    """

    def __init__(self, window_name: str = "vision_processing - get_command"):
        self.window_name = window_name

    def render(self, frame, status: dict) -> bool:
        """frame(np.ndarray)과 status(dict)를 받아 창에 표시합니다.

        Returns:
            bool: 사용자가 'q'를 눌러 종료를 요청했으면 True.
        """
        display_frame = frame.copy()
        self._draw_input_status_overlay(display_frame, status)
        self._draw_status_overlay(display_frame, status)
        cv2.imshow(self.window_name, display_frame)
        key = cv2.waitKey(1) & 0xFF
        return key == ord("q")

    def _draw_input_status_overlay(self, frame, status: dict):
        """화면 상단에 서비스 입력 가능 여부(get_command.py의 _listening/_accumulated
        상태)를 표시합니다.

        get_command.py는 status에 원본 상태만 실어 보냅니다:
          - status['listening'] (bool): robot_control.py가 get_command 서비스를
            호출해 응답을 기다리는 중이면 True. 이미 응답을 보내고 로봇이 그 명령을
            수행 중이면 False.
          - status['accumulated'] (list[str]): _listening=True인 동안 새로 인식된
            제스처 값들이 순서대로 쌓인 목록.

        판단 순서:
          1) listening=True 이고 accumulated 가 비어있으면 -> "[입력 가능]"
          2) listening=True 이고 accumulated 에 값이 있으면
             -> "[입력 중] <accumulated 값을 공백으로 이어붙인 것>"
          3) listening=False 이면(로봇이 동작 중이라 다음 요청을 아직 안 보냄) -> "[입력 불가]"
        """
        listening = status.get("listening", False)
        accumulated = status.get("accumulated") or []

        if listening and not accumulated:
            text = "[Input Enable]"
            color = (0, 255, 0)  # 초록 — 요청 대기 중, 아직 입력 없음
        elif listening and accumulated:
            text = f"[Typing] {' '.join(accumulated)}"
            color = (0, 255, 255)  # 노랑 — 값이 쌓이는 중
        else:
            text = "[Input Diable] Robot Operating"
            color = (0, 0, 255)  # 빨강 — 로봇 동작 중

        w = frame.shape[1]

        bar_height = 36
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, bar_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        cv2.putText(
            frame, text, (10, bar_height - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA,
        )

    def _draw_status_overlay(self, frame, status: dict):
        """화면 하단에 반투명 바를 깔고 진행 상황(커맨드/버퍼 채움 정도/에러)을 표시합니다."""
        h, w = frame.shape[:2]

        if status.get("error") is not None:
            text = f"ERROR: {status['error']}"
            color = (0, 0, 255)  # 빨강 (BGR)
        elif status.get("command") is None:
            text = f"warming up... ({status['buffer_len']}/{status['buffer_maxlen']} frames)"
            color = (0, 165, 255)  # 주황
        else:
            text = (
                f"command: {status['command']}   "
                f"buffer: {status['buffer_len']}/{status['buffer_maxlen']}   "
                f"frames: {status['frame_count']}"
            )
            color = (0, 255, 0)  # 초록

        bar_height = 36
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, h - bar_height), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        cv2.putText(
            frame, text, (10, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA,
        )

    def close(self):
        cv2.destroyWindow(self.window_name)
