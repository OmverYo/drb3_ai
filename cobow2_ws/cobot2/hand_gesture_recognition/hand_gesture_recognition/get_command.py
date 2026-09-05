import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger

from hand_gesture_recognition.video_device import resolve_camera_device
from hand_gesture_recognition.CamController import CamController, CamConfig
from hand_gesture_recognition.vision_processing import CommandRecognizer, SlidingCommandRecognizer
from hand_gesture_recognition.show_webcam import WebcamViewer


PACKAGE_NAME = "hand_gesture_recognition"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
MODEL_PATH = os.path.join(RESOURCE_PATH, "gesture_recognizer_Pjv3_0905_01.task")

DISPLAY_FPS = 30.0

# 조건 1: 숫자 영단어 -> 숫자 기호, 'comma' -> ',' 로 정규화
NUMBER_WORD_TO_DIGIT = {
    "zero": "0",
    "one": "1",
    "two": "2",
    "three": "3",
    "three2": "3",
    "four": "4",
    "five": "5",
    "six": "6",
    "seven": "7",
    "eight": "8",
    "nine": "9",
    "ten": "10",
}
COMMA_WORD = "comma"
COMMA_SYMBOL = ","

# --- 커맨드 시퀀스 형식 검증 ---
# 허용되는 전체 형식(둘 중 하나):
#   [숫자][,][숫자][grap][숫자][,][숫자][release]
#   [숫자][,][숫자][bucket]
# 규칙:
#   1) 형식에 맞지 않는 입력은 그 입력만 무시하고, 같은 자리에서 올바른 다음 입력을 계속 기다린다.
#   2) 콤마로 구분되지 않은 같은 숫자 자리에서는 계속 더해지며, 합이 MAX_NUMBER_VALUE를
#      넘기게 되는 입력은 무시한다(그 입력 전까지의 값을 유지).
#   3) 한 숫자 그룹([숫자][,][숫자])당 콤마는 1개만 허용되고, 그 이후의 추가 콤마는 무시된다.
GRAP_COMMAND = "grap"
RELEASE_COMMAND = "release"
BUCKET_COMMAND = "bucket"
MAX_NUMBER_VALUE = 10

# 형식 문법에 속하지 않는 별도 제어 명령. typing 중(listening=True) 감지되면
# 지금까지 쌓인 _accumulated/형식 상태를 전부 지우고 처음부터 다시 받는다.
RESET_COMMAND = "reset"


class GetCommand(Node):
    # 시퀀스 형식 검증용 상태. 이름은 "이 상태에서 다음에 기대하는 입력"을 의미한다.
    STATE_NUM1_START = "num1_start"   # 첫 번째 숫자 시작 대기
    STATE_NUM1 = "num1"               # 첫 번째 숫자 입력 중 (콤마/추가 숫자 대기)
    STATE_NUM2_START = "num2_start"   # 콤마 뒤 두 번째 숫자 시작 대기
    STATE_NUM2 = "num2"               # 두 번째 숫자 입력 중 (grap/bucket/추가 숫자 대기)
    STATE_NUM3_START = "num3_start"   # grap 뒤 세 번째 숫자 시작 대기
    STATE_NUM3 = "num3"               # 세 번째 숫자 입력 중 (콤마/추가 숫자 대기)
    STATE_NUM4_START = "num4_start"   # 콤마 뒤 네 번째 숫자 시작 대기
    STATE_NUM4 = "num4"               # 네 번째 숫자 입력 중 (release/추가 숫자 대기)
    STATE_DONE = "done"               # 형식 완성(release/bucket 확정) - 이후 입력은 모두 무시

    def __init__(self):
        super().__init__("get_command_node")

        # 윈도우 크기(초)와 슬라이딩 간격(프레임)을 파라미터로 노출 (기본 2초 / 1프레임)
        # 카메라 프레임마다 모델이 라벨 하나(예: fist, open_palm, none)를 뱉는데, 그 라벨들을 최근 몇 초치 모아서 다수결을 낼지 정하는 값입니다.
        # fps(초당 프레임 수)와 곱해서 실제 버퍼 길이(프레임 개수)로 환산됩니다.
        # 예: window_seconds=2.0, fps=30 → 최근 60개 라벨을 보고 다수결.
        # 크면: 순간적으로 손이 흔들리거나 오인식이 한두 프레임 섞여도 잘 안 흔들리고 안정적(노이즈에 강함).
        # 대신 실제 제스처가 바뀌어도 반영되기까지 반응이 느려짐(지연 증가).
        # 작으면: 반응은 빠르지만, 프레임 하나하나의 오인식에 그대로 흔들릴 수 있음.
        self.declare_parameter("window_seconds", 2)
        # 주의: 이 값이 커도 추론 자체는 여전히 매 프레임마다 1번씩 일어납니다(성능 최적화 때문에 이렇게 만들었었죠).
        # 이 값이 조절하는 건 "버퍼가 다 찬 뒤, 최종 결과값(self._latest_command)을 새로고침하는 빈도"입니다.
        # 1(기본값): 새 프레임이 들어올 때마다 매번 다수결을 다시 계산 → 결과가 가장 촘촘하게(실시간으로) 갱신됨.
        # 5: 5프레임(≈0.17초 @30fps)마다 한 번씩만 최종 결과를 갱신 → get_command 서비스가 반환하는 값이 그만큼 덜 자주 바뀜. (다수결 계산 자체는 Counter라 매우 가벼워서, 이 값을 늘리는 주된 이유는 "결과 값이 너무 자주 튀는 걸 원치 않을 때" 정도입니다.)
        self.declare_parameter("slide_frames", 1) # 
        # 카메라 화면을 띄울지 여부 (헤드리스 환경에서는 False로)
        self.declare_parameter("show_window", True)

        window_seconds = self.get_parameter("window_seconds").value
        slide_frames = self.get_parameter("slide_frames").value
        self.show_window = self.get_parameter("show_window").value

        print(PACKAGE_PATH, RESOURCE_PATH, MODEL_PATH)

        recognizer = CommandRecognizer(model_path=MODEL_PATH)

        device_index = resolve_camera_device()
        self.cam_config = CamConfig(device_index=device_index if device_index is not None else 0)

        self.sliding = SlidingCommandRecognizer(
            recognizer,
            window_seconds=window_seconds,
            slide_frames=slide_frames,
            fps=self.cam_config.fps,
        )

        # --- 조건부 응답을 위한 상태 ---
        # _prev_distinct_command: 'none'을 제외하고 마지막으로 관찰된 서로 다른 결과값.
        #   프레임마다 계속 흘러들어오는 값 중 "이전 결과와 같으면 무시"(조건 3)를 위해
        #   요청 여부와 무관하게 항상 갱신되는 전역 상태입니다.
        # _accumulated: 요청이 들어온 뒤(_listening=True) 형식 검증(STATE_NUM1_START ~
        #   STATE_DONE)을 통과한 값들만 순서대로 쌓아두는 목록. 허용 형식은
        #   [숫자][,][숫자][grap][숫자][,][숫자][release] 또는 [숫자][,][숫자][bucket]
        #   뿐이며, 형식에 안 맞는 입력은 버려진다(_apply_format_step 참고).
        #   예) 1 1 1 none , 2 2 2 2 none none , grap 3 , 4 release
        #   -> ["6", ",", "4", "grap", "3", ",", "4", "release"]
        # _qualify_event: release/bucket이 새로 감지되어 응답할 준비가 됐음을 알림.
        self._state_lock = threading.Lock()
        self._prev_distinct_command = None
        self._accumulated = []
        self._listening = False
        self._format_state = self.STATE_NUM1_START
        self._qualify_event = threading.Event()

        self.cam = CamController(self.cam_config, on_frame=self._on_frame)
        self.viewer = WebcamViewer() if self.show_window else None

        self._quit_requested = False

        # 카메라는 노드 생성 시 한 번만 켜서 노드가 살아있는 동안(=패키지 종료까지) 계속 스트리밍합니다.
        try:
            print("open camera stream")
            self.cam.open_stream()
            self.cam.start_streaming()
        except Exception as e:
            self.get_logger().error(f"Error: Failed to open camera stream: {e}")
            raise

        if self.viewer is not None:
            self.get_logger().info("카메라 미리보기 창을 표시합니다. 'q'를 누르면 종료됩니다.")
            # 주의: imshow/waitKey는 메인 스레드에서 호출되어야 하므로, 더 이상 rclpy
            # 타이머로 돌리지 않습니다(그러면 get_command 서비스가 응답을 기다리는 동안
            # 미리보기가 멈춰버립니다). 대신 main()의 메인 스레드 루프에서 직접 호출합니다.

        self.get_logger().info(
            f"GetCommandNode initialized. window={window_seconds}s, slide={slide_frames}frame(s)"
        )
        self.get_logger().info("camera streaming started — 실시간 제스처 분석 진행 중")

        # get_command 서비스 콜백이 판정을 기다리며 블로킹되는 동안에도 다른 콜백이
        # 막히지 않도록 재진입 가능한 콜백 그룹을 사용합니다.
        self.cb_group = ReentrantCallbackGroup()
        self.get_command_srv = self.create_service(
            Trigger, "get_command", self.get_command, callback_group=self.cb_group
        )

    def _on_frame(self, frame):
        """CamController 스트리밍 스레드에서 새 프레임마다 호출됨 (프레임당 추론 1회)."""
        self.sliding.process_frame(frame)
        self._evaluate_vision_result()

    def _normalize_label(self, label):
        """조건 1: 숫자 영단어(one/two/...) -> 숫자 기호, 'comma' -> ',' 기호로 변환."""
        if label in NUMBER_WORD_TO_DIGIT:
            return NUMBER_WORD_TO_DIGIT[label]
        if label == COMMA_WORD:
            return COMMA_SYMBOL
        return label

    @staticmethod
    def _is_numeric_token(token):
        return token.isdigit()

    def _is_valid_digit_token(self, token):
        """숫자이면서 그 자체로 MAX_NUMBER_VALUE(9)를 넘지 않는 토큰인지."""
        return self._is_numeric_token(token) and int(token) <= MAX_NUMBER_VALUE

    def _apply_format_step(self, command):
        """조건 1~3에 맞는 입력만 self._accumulated에 반영하는 상태 기계.

        형식에 어긋나는 입력은 무시하고 같은 상태를 유지한 채 다음 입력을 기다린다
        (조건 3). 숫자 자리는 콤마가 나오기 전까지 계속 합산되며, 합이
        MAX_NUMBER_VALUE(9)를 넘기는 입력은 그 입력만 버린다(조건 2).

        호출 시 self._state_lock을 이미 잡고 있어야 한다.
        반환값: 이번 입력으로 release/bucket까지 유효하게 도달해 시퀀스가
        완성됐으면 True, 아니면 False.
        """
        state = self._format_state
        if state == self.STATE_DONE:
            # 이미 완성된 시퀀스 — 서비스가 응답을 가져가 초기화하기 전까지는 무시.
            return False

        is_digit = self._is_valid_digit_token(command)

        def start_number(next_state):
            self._accumulated.append(command)
            self._format_state = next_state

        def continue_number():
            # 마지막으로 쌓인 숫자에 더한다. 합이 9를 넘으면 이번 입력은 버리고
            # 상태/누적값 모두 그대로 유지한다.
            current = int(self._accumulated[-1])
            added = current + int(command)
            if added > MAX_NUMBER_VALUE:
                return
            self._accumulated[-1] = str(added)

        if state == self.STATE_NUM1_START:
            if is_digit:
                start_number(self.STATE_NUM1)
            # 그 외(숫자가 아님): 조건 1 위반 -> 무시

        elif state == self.STATE_NUM1:
            if is_digit:
                continue_number()
            elif command == COMMA_SYMBOL:
                self._accumulated.append(command)
                self._format_state = self.STATE_NUM2_START
            # grap/release/bucket/기타 -> 무시 (아직 두 번째 숫자 전)

        elif state == self.STATE_NUM2_START:
            if is_digit:
                start_number(self.STATE_NUM2)
            # 콤마 뒤엔 숫자가 와야 함 -> 그 외 무시

        elif state == self.STATE_NUM2:
            if is_digit:
                continue_number()
            elif command == GRAP_COMMAND:
                self._accumulated.append(command)
                self._format_state = self.STATE_NUM3_START
            elif command == BUCKET_COMMAND:
                self._accumulated.append(command)
                self._format_state = self.STATE_DONE
                return True
            # 두 번째 콤마/release/기타 -> 무시

        elif state == self.STATE_NUM3_START:
            if is_digit:
                start_number(self.STATE_NUM3)

        elif state == self.STATE_NUM3:
            if is_digit:
                continue_number()
            elif command == COMMA_SYMBOL:
                self._accumulated.append(command)
                self._format_state = self.STATE_NUM4_START

        elif state == self.STATE_NUM4_START:
            if is_digit:
                start_number(self.STATE_NUM4)

        elif state == self.STATE_NUM4:
            if is_digit:
                continue_number()
            elif command == RELEASE_COMMAND:
                self._accumulated.append(command)
                self._format_state = self.STATE_DONE
                return True
            # 두 번째 콤마/bucket/기타 -> 무시

        return False

    def _evaluate_vision_result(self):
        """조건 1~4를 적용해, 응답할 가치가 있는 새 결과가 나올 때마다 형식 검증을 거쳐
        누적하고, 유효한 release/bucket에 도달하면 대기 중인 서비스 콜백을 깨웁니다.
        RESET_COMMAND가 들어오면 형식 검증 없이 곧바로 누적된 내용을 비웁니다.
        """
        raw_command, error = self.sliding.get_latest()
        if error is not None or raw_command is None:
            # 에러 상태이거나 아직 윈도우가 안 찬 warming-up 상태 — 판단 보류
            return

        command = self._normalize_label(raw_command)

        if command == "none":
            # 조건 2: none은 무시 (이전 결과 비교에도 반영하지 않음)
            return

        with self._state_lock:
            if command == self._prev_distinct_command:
                # 조건 3: 직전 결과와 같으면 무시
                return
            self._prev_distinct_command = command

            if not self._listening:
                # 아무도 응답을 기다리고 있지 않으면 누적하지 않음 (조건 1)
                return

            if command == RESET_COMMAND:
                # 형식 문법에 없는 제어 명령: typing 중이던 내용을 전부 지우고
                # 처음 상태로 되돌린다. _display_loop이 다음 프레임에 이 빈
                # _accumulated를 그대로 읽어가므로 화면 표시도 자동으로 지워진다.
                self._accumulated = []
                self._format_state = self.STATE_NUM1_START
                return

            if self._apply_format_step(command):
                # 형식([숫자],[숫자][grap][숫자],[숫자]release 또는 [숫자],[숫자]bucket)에
                # 맞게 release/bucket까지 유효하게 도달한 경우에만 응답을 확정한다.
                self._qualify_event.set()

    def get_command(self, request, response):
        """조건 1: 요청이 들어온 시점부터 새로 감지되는 결과만 누적해서 기다립니다.
        (요청 이전에 이미 누적돼 있던 값은 버리고 새로 시작)

        release나 bucket이 나오면, 그 전까지 none을 제외하고 누적했던 값들을
        전부 공백으로 이어붙여 한 번에 응답합니다.
        예) 1 1 1 none 2 2 2 2 none none 3 3 release -> "1 2 3 release"
        """
        with self._state_lock:
            self._accumulated = []
            self._format_state = self.STATE_NUM1_START
            self._qualify_event.clear()
            self._listening = True

        while rclpy.ok() and not self._quit_requested:
            if self._qualify_event.wait(timeout=0.1):
                break

        with self._state_lock:
            self._listening = False

        if not rclpy.ok() or self._quit_requested:
            response.success = False
            response.message = "shutdown"
            return response

        with self._state_lock:
            message = " ".join(self._accumulated)
            self._accumulated = []

        response.success = True
        response.message = message
        return response

    def _display_loop(self):
        """메인 스레드 루프에서 주기적으로 호출됩니다 (imshow/waitKey 제약 때문에).

        CamController에서 최신 프레임을, SlidingCommandRecognizer에서 최신 상태를
        가져와 WebcamViewer에 넘겨줍니다. 여기서는 문구를 직접 만들지 않고 원본 상태
        (listening/accumulated)만 그대로 실어 보내고, "무슨 문구를 어떤 색으로 보여줄지"
        판단/그리기는 WebcamViewer(show_webcam.py) 쪽 책임으로 둡니다.

        listening=True 는 robot_control.py가 get_command 서비스를 호출해 응답을
        기다리고 있는 중(=로봇이 다음 명령을 요청한 상태)임을 뜻하고, listening=False 는
        이미 응답을 보내고 로봇이 그 명령을 수행하느라 바쁜 상태(=입력 불가)를 뜻합니다.
        """
        frame = self.cam.get_last_frame()
        if frame is None:
            return

        status = self.sliding.get_status()
        with self._state_lock:
            status["listening"] = self._listening
            status["accumulated"] = list(self._accumulated)
        quit_requested = self.viewer.render(frame, status)
        if quit_requested:
            self.get_logger().info("'q' 입력 — 종료합니다.")
            self._quit_requested = True

    def should_quit(self):
        return self._quit_requested

    def destroy_node(self):
        # 패키지(노드) 종료 시점에 스트리밍을 멈추고 카메라/창을 닫습니다.
        self.cam.stop_streaming()
        self.cam.close_stream()
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = GetCommand()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # imshow/waitKey는 메인 스레드에서만 호출. get_command 서비스가 오래 블로킹돼도
        # executor가 별도 스레드에서 돌기 때문에 미리보기는 계속 갱신됩니다.
        while rclpy.ok() and not node.should_quit():
            if node.viewer is not None:
                node._display_loop()
            time.sleep(1.0 / DISPLAY_FPS)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
