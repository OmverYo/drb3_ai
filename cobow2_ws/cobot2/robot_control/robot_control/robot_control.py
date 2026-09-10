import os
import time
import sys
import json
import math
import argparse
import threading
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import DR_init

from od_msg.srv import SrvDepthPosition, SrvAllPositions, SrvGraspPlan
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG
from robot_control.aruco_calculator import ArucoCalculator
from robot_control.janggi_rules import load_rule_engine
from robot_control.task_json import TaskJsonPublisher, coordinate

package_path = get_package_share_directory("robot_control")

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
BUCKET_POS = [200.58, -26.07, 56.57]#[4.00, 38.00, 64.00, -0.1, 78.0, 4]
JHOME_POS = [0, -30, 90, 0, 90, 0]
PLACE_LIFT = 150.0
PLACE_X_OFFSET = 0.0
PLACE_Y_OFFSET = -8.0
PLACE_Z_OFFSET = -25.0
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -35.0
MIN_DEPTH = 2.0
GRIPPER_PREOPEN_RAW = 480
GRASP_HOVER_MM = 200.0
GRASP_MIN_SCORE = 0.5
GRIPPER_YAW_OFFSET_DEG = float(os.getenv("GRIPPER_YAW_OFFSET_DEG", "0"))

# after(놓을 자리) 점유 판별용 반경(mm). 장기말은 대략 원형이라, after 위치를
# 중심으로 이 반경 안에 다른 detection 후보가 있으면 "이미 다른 말이 있다"고 본다.
# 실제 말 크기에 맞춰 조정 가능.
PIECE_MATCH_RADIUS_MM = 15.0

# before(집는 위치) 판정용 최대 허용 거리(mm). 음성/비전 명령으로 지정한 칸(아루코
# 계산 위치)과 실제 detection 사이 거리가 이 값을 넘으면, "그 칸에는 실제로 말이
# 없는데 엉뚱하게 옆 칸의 말을 집으려 한다"고 보고 로봇 동작을 하지 않는다.
# 반 칸(half-cell) 정도로 설정하는 것을 권장 - 보드 격자 간격(칸 크기)의 절반으로
# 실측값에 맞게 조정할 것.
BEFORE_PICK_MATCH_RADIUS_MM = 17.5

# --- 비전 서비스(get_command) 관련 설정 ---
# 음성(get_keyword)과 달리 별도의 "시작 신호"가 없다. get_command.py 노드가 뜨는 순간
# 웹캠 스트리밍이 자동으로 시작되므로, 여기서도 노드 생성 직후부터 곧바로(그리고 계속)
# get_command 서비스를 폴링한다.
VISION_POLL_PERIOD_SEC = 0.2

# _on_vision_response 안에서 발생하는, 위치 관련 "과정적" 메시지(감지된 장기말 없음/
# 좌표 해석 불가/규칙 위반/잘못된 착수/상대 기물 포획)를 별도 alert 창이 아니라
# get_command.py가 띄우는 webcam 화면에 잠깐 표시하기 위한 지속 시간(초).
# (결론적 메시지인 "vision 이동 실패" 등에는 적용하지 않는다.)
VISION_STATUS_DISPLAY_SEC = 2.0
VISION_STATUS_TOPIC = "/ui/vision_status"

# --- 실행 모드 ---
# voice(기본): get_keyword(음성) + get_position(depth) 서비스로 pick-and-place 실행.
# vision: get_command(손동작) 서비스만 폴링(음성/깊이 서비스는 사용하지 않음).
# ROS2가 뒤에 붙이는 '--ros-args ...'와 충돌하지 않도록 parse_known_args를 사용한다.
def _parse_mode():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--mode",
        choices=["voice", "vision"],
        default="voice",
        help="voice(기본): 음성+깊이 서비스로 pick-and-place. vision: 손동작 인식 서비스만 폴링.",
    )
    known_args, _ = parser.parse_known_args()
    return known_args.mode


MODE = _parse_mode()


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


class GraspPlanError(RuntimeError):
    """The planner rejected the grasp before the robot started descending."""


class RobotController(Node):
    def __init__(self, mode: str = MODE):
        super().__init__("pick_and_place")
        self.mode = mode

        # json 기록용 pulisher. task_json.py 내부에서 JSON 파일을 기록/전송
        self.task_json = TaskJsonPublisher(self)
        

        # MultiThreadedExecutor 하에서 서비스 응답 콜백/타이머 콜백이 서로 블로킹 없이
        # 동시에 처리될 수 있도록 재진입 가능한 콜백 그룹을 사용한다.
        self.cb_group = ReentrantCallbackGroup()
        self.board_sync_client = self.create_client(
            SetBool, "/set_board_sync", callback_group=self.cb_group
        )

        # 안전 회전각을 찾기 위한 grasp plan 서비스.
        self.get_grasp_plan_client = self.create_client(
            SrvGraspPlan, "/get_grasp_plan", callback_group=self.cb_group)

        self.gripper2cam_path = os.path.join(package_path,"resource","T_gripper2camera.npy")

        self.aruco_calculator = ArucoCalculator(node=self,
            get_current_posx_fn=get_current_posx,
            t_gripper_camera_path=self.gripper2cam_path,
            )

        # 장기 규칙(이동 제한) 엔진. 규칙은 janggi_move_rules.yaml 에서 관리하며,
        # JANGGI_RULES_PATH 환경변수로 경로를 override 할 수 있다(load_rule_engine 참고).
        self.rule_engine = load_rule_engine()
        # (row0, col0) -> BASE xyz 캐시. 보드/궁성은 고정되어 있으므로 최초 1회만 계산.
        self._cell_position_cache = None

        self.get_logger().info(f"RobotController mode = '{self.mode}'")

        self.isBucket = False

        # voice/vision 공통: UI 알림 퍼블리셔(잘못된 착수 알림 등에도 사용).
        self.ui_pub = self.create_publisher(String, "/ui/current_task", 10)
        self._publish_task(None, None)

        if self.mode == "voice":
            self._init_voice_services()
        else:
            self._init_vision_service()

    def _init_voice_services(self):
        """옵션 1(기본): get_keyword(음성) + get_position(depth) 서비스만 준비한다."""
        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position", callback_group=self.cb_group
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            pass
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(
            Trigger, "/get_keyword", callback_group=self.cb_group
        )
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            pass
        self.get_keyword_request = Trigger.Request()

    def _init_vision_service(self):
        """옵션 2: get_command(손동작 인식) 서비스만 준비하고 계속 폴링한다."""
        self.vision_client = self.create_client(
            Trigger, "get_command", callback_group=self.cb_group
        )
        while not self.vision_client.wait_for_service(timeout_sec=3.0):
            pass
        self.vision_request = Trigger.Request()

        # 아루코로 계산한 board_xyz_before(집는 위치)를 실제 detection 결과로
        # 보정하기 위한 서비스. 클래스 무관, 전체 detection 후보 중
        # 아루코 계산값과 가장 가까운 것을 robot_control 쪽에서 선택한다.
        self.get_all_positions_client = self.create_client(
            SrvAllPositions, "/get_all_positions", callback_group=self.cb_group
        )
        while not self.get_all_positions_client.wait_for_service(timeout_sec=3.0):
            pass
        self.get_all_positions_request = SrvAllPositions.Request()

        # _on_vision_response의 위치 관련 과정적 메시지(감지된 장기말 없음/좌표 해석
        # 불가/규칙 위반/잘못된 착수/상대 기물 포획)를 get_command.py의 webcam 화면에
        # 잠깐 띄우기 위한 퍼블리셔. 로봇 동작에는 영향을 주지 않는 UI 전용 채널이다.
        self.vision_status_pub = self.create_publisher(
            String, VISION_STATUS_TOPIC, 10
        )

        # 조건 3: 요청 전송~응답 처리(z축 bump 이동 포함) 동안 True.
        # 다음 요청은 이 플래그가 False로 돌아온 뒤에만 나간다.
        self._vision_busy = False
        self._vision_lock = threading.Lock()

        # 조건 1: 별도 시작 신호 없이, 노드가 뜬 시점부터 타이머로 계속 폴링.
        self.vision_timer = self.create_timer(
            VISION_POLL_PERIOD_SEC,
            self._request_vision_command,
            callback_group=self.cb_group,
        )
        self.vision_timer.cancel()  # 폴링은 노드 생성 직후부터 시작되므로, 여기서는 타이머를 일단 멈춰둔다.

    def _publish_task(self, target, pos):
        data = {}
        if target:
            data["target"] = target
        if pos:
            data["pos"] = pos
        try:
            self.ui_pub.publish(String(data=json.dumps(data)))
        except Exception as e:
            self.get_logger().warn(f"_publish_task failed (non-critical): {e}")

    def _publish_alert(self, message):
        """잘못된 착수 등, 사용자에게 즉시 보여줘야 하는 알림을 UI 토픽으로 보낸다."""
        try:
            self.ui_pub.publish(String(data=json.dumps({"alert": message})))
        except Exception as e:
            self.get_logger().warn(f"_publish_alert failed (non-critical): {e}")

    def _publish_vision_status(self, message, duration_sec=VISION_STATUS_DISPLAY_SEC):
        """_on_vision_response의 위치 관련 과정적 메시지를 get_command.py(webcam 화면)
        쪽으로 보내, 그 화면에 duration_sec 동안만 잠깐 표시되도록 한다.
        (결론적 실패 메시지인 "vision 이동 실패" 등에는 사용하지 않는다.)
        """
        try:
            self.vision_status_pub.publish(
                String(data=json.dumps({"message": message, "duration": duration_sec}))
            )
        except Exception as e:
            self.get_logger().warn(f"_publish_vision_status failed (non-critical): {e}")

    def _wait_for_future(self, future, timeout_sec=None):
        """MultiThreadedExecutor가 별도 스레드에서 이미 spin 중이므로,
        여기서는 spin_until_future_complete 대신 완료될 때까지 폴링만 한다.
        (같은 노드를 두 곳에서 동시에 spin하는 것을 피하기 위함)
        """
        start = time.time()
        while rclpy.ok() and not future.done():
            if timeout_sec is not None and (time.time() - start) > timeout_sec:
                return False
            time.sleep(0.01)
        return future.done()

    def set_board_sync(self, enable):
        '''False 보내면 현황판 중단, True 보내면 현황판 재개'''
        if not self.board_sync_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/set_board_sync service not available')

        request = SetBool.Request()
        request.data = enable
        future = self.board_sync_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=15.0):
            future.cancel()
            raise RuntimeError('/set_board_sync service call timed out')

        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('현황판 상태 변경 실패')

    # ------------------------------------------------------------------
    # 비전 서비스(get_command) 폴링
    # ------------------------------------------------------------------
    def _request_vision_command(self):
        """타이머 콜백. 이전 요청(및 그에 따른 bump 이동)이 아직 끝나지 않았으면
        (조건 3) 이번 틱은 건너뛴다.
        """
        with self._vision_lock:
            if self._vision_busy:
                return
            self._vision_busy = True

        future = self.vision_client.call_async(self.vision_request)
        future.add_done_callback(self._on_vision_response)

    def _on_vision_response(self, future):
        """조건 2: 응답 성공/실패, 인식된 커맨드 값에서 현재 위치와 이동할 위치값 추출하여 이동

        추가된 규칙(장기 규칙 반영):
        - after(놓을 자리)에 이미 다른 말이 있는지, 그 위치의 detection 후보들과
          before 말의 클래스(편)를 비교해서 판별한다.
        - 같은 편(예: 둘 다 *_red)이면 잘못된 착수이므로 알림만 띄우고 동작을 취소한다.
        - 다른 편(상대 말)이면 포획: after 자리의 상대 말을 먼저 bucket으로 옮긴 뒤,
          이어서 before -> after 이동을 수행한다.
        - after 위치가 비어 있으면 기존과 동일하게 곧바로 before -> after 이동만 수행한다.
        """
        board_xyz_before = None
        board_xyz_after = None
        capture_target = None  # after 위치를 미리 점유하고 있는 상대 말(포획 대상)
        abort_move = False
        text_split = None
        task_started = False
        task_error = "Vision move rejected"

        try:
            result = future.result()
            if result is not None and result.success:
                # result.message example: "1 , 3 grap 3 , 4 release"
                text_split = result.message.split(' ')

                board_pos_before = f'{text_split[0]},{text_split[2]}'
                self._record_task('start', before=board_pos_before,
                    after=f'{text_split[4]},{text_split[6]}' if text_split[-1] == 'release' else None)
                task_started = True
                # 이때 get_board_target_pos 내부에서 계산시 z 값은 realsense depth 카메라로 부터 받아서 사용해야 하므로, 필수로 켜줘야 함.
                #1행 1열 부터 시작하는 텍스트 '(row,colunm)' 형태로 받아서 좌표값 xyz 로 반환. 
                board_xyz_before = self.get_board_target_pos(board_pos_before)
                if board_xyz_before is None:
                    self.get_logger().warn(f"Invalid board target(before): {board_pos_before}")
                    abort_move = True
                else:
                    # realsense 값 그대로 사용이 안됨. aruco 계산 시 보정 필요.
                    # -> 아루코 추정 위치 근처에서 실제 detection 결과를 찾아 대체
                    #    (클래스 무관, 최근접). 이때 그 후보의 클래스명도 함께 받아둔다
                    #    (before 말의 편을 알아야 after 자리 점유 판별이 가능하므로).
                    # max_dist(BEFORE_PICK_MATCH_RADIUS_MM)를 벗어나는 detection은
                    # "다른 칸의 말"로 보고 무시한다 -> 반경 안에 아무 것도 없으면
                    # (None, None)을 돌려받아 "빈 칸을 집으려 한 것"으로 처리한다.
                    board_xyz_before, before_name = self.refine_board_pos_with_detection(
                        board_xyz_before, max_dist=BEFORE_PICK_MATCH_RADIUS_MM
                    )

                    if board_xyz_before is None:
                        self._publish_vision_status("error : no piece detected")
                        abort_move = True
                    else:
                        board_xyz_before[0] = board_xyz_before[0] #+ PLACE_X_OFFSET
                        board_xyz_before[1] = board_xyz_before[1] + PLACE_Y_OFFSET
                        board_xyz_before[2] = 3

                        self.isBucket = False

                        # after 위치는 판 내부 or 버킷(딴 상대방 말)
                        if text_split[-1] == 'release':
                            board_pos_after = f'{text_split[4]},{text_split[6]}'
                            board_xyz_after = self.get_board_target_pos(board_pos_after)
                            if board_xyz_after is None:
                                self._publish_vision_status("error : invalid board target")
                                self.get_logger().warn(f"Invalid board target(after): {board_pos_after}")
                                abort_move = True
                            else:
                                # --- 장기 규칙(이동 제한) 검사 ---
                                # before_name(예: cha_green)의 기물 타입에 맞는 이동 규칙
                                # (janggi_move_rules.yaml)을 적용해, 이 before->after 이동이
                                # 실제 장기 규칙상 유효한지(예: 차라면 경로 위에 다른 말이
                                # 없어야 함) 검사한다. 위반 시 로봇 동작을 아예 수행하지 않는다.
                                before_parsed = self.parse_board_destination(board_pos_before)
                                after_parsed = self.parse_board_destination(board_pos_after)
                                if before_parsed is None or after_parsed is None:
                                    self._publish_vision_status("error : recognition failure")
                                    abort_move = True
                                else:
                                    before_rc = before_parsed[2:4]
                                    after_rc = after_parsed[2:4]
                                    occupancy_grid = self.build_occupancy_grid()
                                    move_result = self.rule_engine.validate_move(
                                        before_name, before_rc, after_rc, occupancy_grid
                                    )
                                    if not move_result.ok:
                                        self._publish_vision_status("error : rule violation")
                                        abort_move = True

                            if abort_move:
                                pass
                            else:
                                # after 위치를 이미 점유하고 있는 말이 있는지 검사.
                                # (타겟 크기만큼의 반경 안에 다른 detection이 겹치면 점유로 판단)
                                occupant = self.find_piece_at(board_xyz_after)
                                if occupant is not None:
                                    before_team = (
                                        before_name.rsplit('_', 1)[-1] if before_name else None
                                    )
                                    occupant_team = occupant['name'].rsplit('_', 1)[-1]
                                    if before_team is not None and occupant_team == before_team:
                                        self._publish_vision_status("error : invalid move")
                                        abort_move = True
                                    else:
                                        self._publish_vision_status("info : capture piece")
                                        capture_target = occupant
                        elif text_split[-1] == 'bucket':
                            self.isBucket = True
                            board_xyz_after = BUCKET_POS
                        else:
                            self.get_logger().warn(
                                f"Unknown vision command suffix: {text_split[-1]}"
                            )
                            abort_move = True
            else:
                abort_move = True
        except Exception as e:
            self.get_logger().error(f"vision 서비스 응답 처리 실패: {e}")
            task_error = str(e)
            abort_move = True

        if abort_move or board_xyz_before is None or board_xyz_after is None:
            if task_started:
                self._record_task('failed', error=locals().get('msg', task_error))
            self.isBucket = False
            # 잘못된 착수/파싱 실패 등 - 아무 동작도 수행하지 않고 다음 요청을 받는다.
            with self._vision_lock:
                self._vision_busy = False
            return

        try:
            #이전 pos 는 쓰잘때기 없는? 회전 값까지 요구하므로, 이를 결국 제자리 값인 0'-180'-0' 로 회전하도록 == 회전 안하도록 줌.
            board_xyzRyRzRy_before = [float(board_xyz_before[0]), float(board_xyz_before[1]), float(board_xyz_before[2])] + [0.0, 180.0, 0.0]

            if capture_target is not None:
                # 상대 말을 먼저 bucket으로 이동(포획)한 다음, 원래 이동을 이어서 수행.
                capture_pos = [
                    float(capture_target['pos'][0]),
                    float(capture_target['pos'][1]) + PLACE_Y_OFFSET,
                    3.0,
                ] + [0.0, 180.0, 0.0]
                self.isBucket = True
                self.pick_and_place_target(capture_pos, BUCKET_POS)
                # 원래 목적지가 bucket이 아니었다면 isBucket 상태를 되돌려 놓는다.
                self.isBucket = (text_split[-1] == 'bucket')

            #after 값은 pick_and_place_target() 내부에서 before 처럼 변환 수행하므로 그대로 넣어줌.
            self.pick_and_place_target(board_xyzRyRzRy_before, board_xyz_after)
            self.init_robot()
            self._record_task("completed" if rclpy.ok() else "failed")
        except Exception as e:
            self.get_logger().error(f"vision 이동 실패: {e}")
            if task_started:
                self._record_task("failed", error=str(e))
        finally:
            self.isBucket = False
            # 이동이 끝난 뒤에야 다음 요청을 허용 (조건 3)
            with self._vision_lock:
                self._vision_busy = False


    def _record_task(self, action, **fields):
        # [JSON 추가] 기록 오류 처리는 여기서만 하고 로봇 동작으로 전파하지 않는다.
        try:
            if action == 'start':
                for key in ('before', 'after'):
                    if isinstance(fields.get(key), str):
                        fields[key] = coordinate(fields[key])
                self.task_json.start(**fields)
            else:
                self.task_json.finish(status=action, **fields)
        except Exception:
            self.get_logger().warn('Task JSON 기록 실패')

     # 처음 정한 파지 XYZ는 유지하고, detection에 안전한 회전 방향을 물어봐서 최종 파지 자세를 만드는 함수 이 함수 자체는 로봇을 움직이지 않습니다.
    def get_grasp_plan(self, target, reference_pos):
        capture_pose = np.asarray(get_current_posx()[0], float)
        hand_eye = np.load(self.gripper2cam_path)
        T = self.get_robot_pose_matrix(*capture_pose) @ hand_eye
        request = SrvGraspPlan.Request()
        request.target = str(target).split('@', 1)[0]
        request.min_score = GRASP_MIN_SCORE
        request.use_reference = True
        request.reference_base_xy_mm = [float(v) for v in reference_pos[:2]]
        request.base_from_camera = T.reshape(-1).tolist()
        request.max_target_distance_mm = float(os.getenv('GRASP_TARGET_GATE_MM', '15'))
        request.ambiguity_margin_mm = float(os.getenv('GRASP_TARGET_AMBIGUITY_MM', '5'))
        if not self.get_grasp_plan_client.wait_for_service(timeout_sec=3.0):
            raise GraspPlanError('/get_grasp_plan unavailable; no descent')
        future = self.get_grasp_plan_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=20.0) or not rclpy.ok():
            future.cancel()
            raise GraspPlanError('Grasp plan timed out; no descent')
        result = future.result()
        if result is None or not result.success:
            raise GraspPlanError(
                'Grasp plan failed: ' + (result.message if result else 'no response')
            )
        # SAM이 선택한 말이 기존 목표 위치에서 너무 멀지 않은지만 확인
        if ( not np.isfinite(result.target_distance_mm)or result.target_distance_mm > request.max_target_distance_mm):
            raise GraspPlanError('SAM target outside target gate')
        # 수직 하강 방향인지 확인
        approach_base = (T[:3, :3]@ np.asarray(result.approach_axis_camera, dtype=float))
        if ( not np.all(np.isfinite(approach_base))or not np.allclose(approach_base, [0, 0, -1], atol=0.02)):
            raise GraspPlanError('Planner approach differs from Base vertical')
        # 수직 하강 방향인지 확인
        if (not np.isfinite(result.clearance_mm)or result.clearance_mm <= 0):
            raise GraspPlanError('Invalid clearance')
        # 최초 XYZ를 유지하고 회전 방향만 반영한다.
        rx, ry, rz = self._grasp_yaw_to_base_euler(result.closing_axis_camera,capture_pose)
        pose = [
            float(reference_pos[0]),float(reference_pos[1]),float(reference_pos[2]),rx,ry,rz
        ]
        self.get_logger().info(
            f"Grasp ready: target={result.detected_name}, "
            f"yaw={result.safe_yaw_deg:.1f}deg, clearance={result.clearance_mm:.1f}mm"
        )
        return pose
    
    # closing_axis_camera(카메라 기준 "닫히는 방향" 벡터)를 캘리브레이션 행렬(gripper2cam)과 현재 로봇 자세로 Base 좌표계로 회전시킨 뒤, 
    # 그 벡터가 가리키는 방향을 Euler ZYZ 회전으로 변환한다. 
    # 그리퍼 손가락은 180도 대칭(어느 쪽으로 돌아도 결과가 같음)이므로 두 후보 중 현재 자세에서 덜 도는 쪽을 골라 불필요한 큰 회전을 피함
    def _grasp_yaw_to_base_euler(self, closing_axis_camera, robot_posx):
        R = (self.get_robot_pose_matrix(*robot_posx) @ np.load(self.gripper2cam_path))[:3,:3]
        axis = R @ np.asarray(closing_axis_camera,float)
        if not np.all(np.isfinite(axis)) or np.linalg.norm(axis[:2]) < 0.9 or abs(axis[2]) > 0.05:
            raise ValueError('Invalid/non-horizontal closing axis')
        phi = math.degrees(math.atan2(axis[1],axis[0])) + GRIPPER_YAW_OFFSET_DEG
        # Explicit ZYZ at beta=180 avoids singular Euler decomposition.
        # alpha-180 is tool X heading. The fingers have 180-degree symmetry.
        candidates = [phi-180, phi]
        current_R = self.get_robot_pose_matrix(*robot_posx)[:3,:3]
        def rotation_cost(alpha):
            next_R = self.get_robot_pose_matrix(0,0,0,alpha,180,0)[:3,:3]
            return -float(np.trace(current_R.T @ next_R))
        alpha = min(candidates,key=rotation_cost)
        alpha = (alpha+180)%360-180
        return float(alpha),180.0,0.0


    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]
    
    def parse_board_destination(self,dest):
        if dest is None:
            return None

        dest = str(dest).strip()

        if "," not in dest:
            self.get_logger().warn("Invalid board destination format: '{dest}'")
            return None
        try:
            row_text, col_text = dest.split(",",1)
            row_user = int(row_text.strip())
            col_user = int(col_text.strip())
        except ValueError:
            self.get_logger().warn(f"Board destination is not integer: '{dest}'")
            return None

        # 사용자 좌표 검사
        if not (1 <= row_user <= 10 and 1 <= col_user <= 9):
            self.get_logger().warn(f"Board destination out of range: {row_user},{col_user}")
            return None

        # 내부 0-based 좌표
        row0 = row_user - 1
        col0 = col_user - 1

        return row_user, col_user, row0, col0

    def get_board_target_pos(self,dest):
        parsed = self.parse_board_destination(dest)
        if parsed is None:
            return None
        (row_user,col_user,row0,col0) = parsed

        board_xyz = self.aruco_calculator.get_base_point(row0,col0)

        if board_xyz is None:
            self.get_logger().warn("Aruco board BASE coordinate calculation failed")
            return None

        return board_xyz

    def robot_control(self):
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        self._wait_for_future(get_keyword_future, timeout_sec=60.0)
        if not rclpy.ok():
            return
        if get_keyword_future.result() is not None and get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()

            message = get_keyword_result.message
            if "/" in message:
                obj_part, dst_part = message.split("/", 1)
                tools = obj_part.split()
                dests = dst_part.split()
            else:
                tools = message.split()
                dests = []

            for i, target in enumerate(tools):
                dest = dests[i] if i < len(dests) else None
                self._record_task('start', piece=target, after=dest)
                try:  # 이제 받은 target/dest를 실제로 집고 놓는 동작 수행을 하고 로봇 동작이 끝나면 task_json에 기록한다.
                    self._publish_task(target, dest)
                    board_xyz = self.get_board_target_pos(dest)
                    if board_xyz is None:
                        self.get_logger().warn(f"Invalid board target: {dest}")
                        self._record_task("failed", error="Target or destination unavailable")
                        continue
                    target_pos = self.get_target_pos(target)
                    if target_pos is None:
                        self._record_task("failed", error="Target or destination unavailable")
                        continue
                    self.pick_and_place_target(target_pos, board_xyz, grasp_selector=target)
                    self.init_robot()
                    self._record_task("completed" if rclpy.ok() else "failed")
                except GraspPlanError as e:
                    self._record_task('failed', error=str(e))
                    self.get_logger().warn(str(e))
                    continue
                except Exception as e:
                    self._record_task('failed', error=str(e))
                    raise

            self._publish_task(None, None)

        else:
            # get_keyword 는 실패 사유를 message 에 담아 돌려준다(성공 시엔 키워드).
            result = get_keyword_future.result()
            reason = result.message if result is not None else "no_response"
            self.get_logger().warn(f"get_keyword 실패: {reason or 'no keyword detected'}")
            if reason == "openai_quota_exhausted":
                self.get_logger().error(
                    "OpenAI 크레딧 소진 — 충전 필요: "
                    "https://platform.openai.com/settings/organization/billing"
                )
            return

    def get_target_pos(self, target):
        target_pos = None
        self.get_position_request.target = target
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        self._wait_for_future(get_position_future)
        if not rclpy.ok():
            return None

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            if sum(result) == 0:
                self.get_logger().warn("No target position")
                return None

            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, self.gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)

            target_pos = list(td_coord[:3]) + robot_posx[3:]
        return target_pos

    def _get_all_detections_base(self, min_score=0.0):
        """get_all_positions 서비스로 현재 화면의 모든 detection 후보를 가져와
        BASE 프레임 좌표로 변환한 뒤 리스트로 반환한다.
        각 원소는 {'pos': np.array([x,y,z]), 'name': str, 'score': float}.
        서비스 응답이 없거나 후보가 없으면 빈 리스트를 반환한다.
        refine_board_pos_with_detection()과 find_piece_at()이 공통으로 사용한다.
        """
        self.get_all_positions_request.min_score = min_score
        future = self.get_all_positions_client.call_async(self.get_all_positions_request)
        self._wait_for_future(future, timeout_sec=5.0)
        if not rclpy.ok():
            return []

        result = future.result()
        if result is None or len(result.x) == 0:
            return []

        robot_posx = get_current_posx()[0]
        names = list(result.name) if len(result.name) == len(result.x) else [None] * len(result.x)

        detections = []
        for cam_x, cam_y, cam_z, score, name in zip(
            result.x, result.y, result.z, result.score, names
        ):
            base_xyz = self.transform_to_base(
                [cam_x, cam_y, cam_z], self.gripper2cam_path, robot_posx
            )
            detections.append({
                "pos": np.asarray(base_xyz, dtype=float),
                "name": name,
                "score": float(score),
            })
        return detections

    def refine_board_pos_with_detection(self, reference_xyz, max_dist=None):
        """아루코 기반 board_xyz(reference_xyz, BASE 프레임)와 가장 가까운
        실제 detection 결과를 찾아 (BASE 프레임 좌표, 클래스명)을 반환한다.
        클래스 무관, 화면에 보이는 모든 detection 후보 중 최근접을 사용한다.

        max_dist: None이면 거리 제한 없이 최근접 후보를 그대로 사용한다(과거 동작과 동일).
                  값이 주어지면, 최근접 후보와의 거리가 이 값(mm)을 넘거나 후보가 아예
                  없을 때 "그 위치엔 실제 말이 없다"고 판단해 (None, None)을 반환한다.
                  before(집는 위치) 판정처럼 "빈 칸을 잘못 지정했는지" 걸러내야 하는
                  경우 반드시 max_dist를 지정해서 호출할 것.
        """
        if reference_xyz is None:
            return None, None

        detections = self._get_all_detections_base()
        if not detections:
            if max_dist is not None:
                self.get_logger().warn("get_all_positions: 후보 없음. 빈 칸으로 판단합니다.")
                return None, None
            self.get_logger().warn("get_all_positions: 후보 없음. 아루코 계산값 사용.")
            return reference_xyz, None

        ref = np.array(reference_xyz[:3], dtype=float)
        best = None
        best_dist = None
        for det in detections:
            dist = float(np.linalg.norm(det["pos"] - ref))
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best = det

        if best is None or (max_dist is not None and best_dist > max_dist):
            if max_dist is not None:
                self.get_logger().warn(
                    f"get_all_positions: 반경 {max_dist}mm 이내에 유효 후보 없음"
                    f"(best_dist={best_dist}). 빈 칸으로 판단합니다."
                )
                return None, None
            self.get_logger().warn(
                f"get_all_positions: 유효 후보 없음(best_dist={best_dist}). 아루코 계산값 사용."
            )
            return reference_xyz, None

        return list(best["pos"]), best["name"]

    def find_piece_at(self, position, radius=PIECE_MATCH_RADIUS_MM):
        """position(BASE 프레임 [x, y, z...])을 중심으로 radius(mm) 안에 있는
        detection 후보 중 가장 가까운 것을 반환한다: {'pos', 'name', 'score'}.
        범위 안에 아무 후보도 없으면 None.

        타겟(장기말)을 after 위치에 놓으려 할 때, 그 자리에 이미 다른 말이
        있는지(=범위 내 겹침)를 판별하는 용도.
        """
        detections = self._get_all_detections_base()
        if not detections:
            return None

        ref = np.array(position[:3], dtype=float)
        best = None
        best_dist = None
        for det in detections:
            dist = float(np.linalg.norm(det["pos"] - ref))
            if dist <= radius and (best_dist is None or dist < best_dist):
                best_dist = dist
                best = det
        return best

    def _get_cell_positions(self):
        """{(row0, col0): BASE xyz(np.array)} 전체 보드 칸 좌표 캐시.

        보드/카메라가 고정되어 있다는 전제 하에 최초 호출 시 1회만 계산한다.
        아루코 마커가 흔들리거나 카메라가 재조정된 경우 self._cell_position_cache = None
        으로 초기화하면 다음 호출에서 다시 계산한다.
        """
        if self._cell_position_cache is not None:
            return self._cell_position_cache

        cache = {}
        rows = self.rule_engine.rows
        cols = self.rule_engine.cols
        for row0 in range(rows):
            for col0 in range(cols):
                base_xyz = self.aruco_calculator.get_base_point(row0, col0)
                if base_xyz is None:
                    continue
                cache[(row0, col0)] = np.asarray(base_xyz[:3], dtype=float)

        if not cache:
            self.get_logger().warn("보드 칸 좌표 캐시 생성 실패(아루코 계산값 없음).")
            return {}

        self._cell_position_cache = cache
        return cache

    def build_occupancy_grid(self):
        """현재 화면의 모든 detection을 (row0, col0) 보드 칸으로 스냅한 occupancy dict를 만든다.

        {(row0, col0): {'pos':..., 'name':..., 'score':...}} 형태이며, 빈 칸은 키가 없다.
        JanggiRuleEngine.validate_move()의 occupancy 인자로 그대로 사용한다.
        """
        detections = self._get_all_detections_base()
        cell_positions = self._get_cell_positions()
        return self.rule_engine.build_occupancy_from_detections(
            detections, cell_positions, match_radius=PIECE_MATCH_RADIUS_MM
        )

    def init_robot(self):
        
        JReady = [-13, 21, 43, 0, 115.5, -13]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

        self.set_board_sync(True)

    def pick_and_place_target(self, target_pos, board_xyz, grasp_selector="*"):
        self.set_board_sync(False)
        lift_pos = list(target_pos[:2]) + [target_pos[2] + GRASP_HOVER_MM, 0., 180., 0.]
        movel(lift_pos, vel=VELOCITY, acc=ACC)
        mwait()
        time.sleep(0.2)
        try:
            target_pos = self.get_grasp_plan(grasp_selector, target_pos)
        except GraspPlanError:
            # Planning is performed at the 200 mm hover pose. Return home and
            # resume board sync without ever descending toward the piece.
            self.init_robot()
            raise
        lift_pos = list(target_pos[:2]) + [lift_pos[2]] + target_pos[3:]
        movel(lift_pos, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.move_gripper(GRIPPER_PREOPEN_RAW)
        # 그리퍼 열림 상태가 되기까지 최대 10초 기다린다. (그리퍼가 이미 열려있으면 바로 다음 단계로 넘어간다)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and gripper.get_status()[0]:
            if time.monotonic() > deadline:
                raise RuntimeError('Gripper opening timed out; no descent')
            time.sleep(0.05)
        if not rclpy.ok():
            raise RuntimeError('ROS shutdown requested; no descent')
        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper()

        while rclpy.ok() and gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()

        
        movel(lift_pos, vel=VELOCITY, acc=ACC)
        mwait()

        hover_pos = [float(board_xyz[0] + PLACE_X_OFFSET),float(board_xyz[1] + PLACE_Y_OFFSET), PLACE_LIFT,] + target_pos[3:]
        if self.isBucket :
            place_pos = [float(board_xyz[0] + PLACE_X_OFFSET),float(board_xyz[1] + PLACE_Y_OFFSET), BUCKET_POS[2], ] + target_pos[3:]
        else :
            place_pos = [float(board_xyz[0] + PLACE_X_OFFSET),float(board_xyz[1] + PLACE_Y_OFFSET), 4, ] + target_pos[3:]
        movel(hover_pos, vel=VELOCITY, acc=ACC)
        mwait()


        movel(place_pos, vel=VELOCITY, acc=ACC)
        mwait()

        gripper.open_gripper()
        while rclpy.ok() and gripper.get_status()[0]:
            time.sleep(0.5)

        movel(hover_pos, vel=VELOCITY, acc=ACC)
        mwait()
        
         


def main(args=None):
    node = RobotController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.init_robot()
        if node.mode == "vision":
            node.vision_timer.reset()  # 폴링 타이머 시작
        if node.mode == "voice":
            while rclpy.ok():
                node.robot_control()
        else:
            # vision 모드는 vision_timer 콜백이 백그라운드에서 계속 get_command를
            # 폴링/처리하므로, 메인 스레드는 별도 반복 호출 없이 대기만 하면 된다.
            while rclpy.ok():
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        executor_thread.join(timeout=2.0)
        node._record_task("failed", error="Robot stopped before task completion")
        node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()