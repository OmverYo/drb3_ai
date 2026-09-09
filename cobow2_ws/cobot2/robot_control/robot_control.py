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
from robot_control.task_json import TaskJsonPublisher, coordinate

package_path = get_package_share_directory("robot_control")

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
BUCKET_POS = [200.58, -26.07, 56.57]#[4.00, 38.00, 64.00, -0.1, 78.0, 4]
JHOME_POS = [0, -30, 90, 0, 90, 0]
PLACE_LIFT = 200.0
PLACE_X_OFFSET = 0.0
PLACE_Y_OFFSET = -8.0
PLACE_Z_OFFSET = -25.0
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -35.0
MIN_DEPTH = 2.0

# --- SAM 파지 각도(yaw) 보정 관련 설정 ---
# hover 상태에서 /get_grasp_plan 서비스가 반환하는 후보 중, 이 점수 미만은
# 파지 대상으로 선택하지 않는다(주변 말을 장애물로 넣는 것과는 무관).
GRASP_MIN_SCORE = float(os.getenv('GRASP_MIN_SCORE', '0.5'))
# 파지 계획이 반환하는 closing_axis_camera는 카메라 로컬 좌표계 기준이다.
# BASE 프레임으로 회전 변환한 뒤, "rz=0일 때 툴 로컬 X축 = 집게가 열리고
# 닫히는 축"이라고 가정하고 yaw를 계산한다. 실제 장착 각도가 이 가정과
# 어긋나면(예: 집게 축이 90도 돌아가 있음) 여기 오프셋으로 보정한다.
GRIPPER_YAW_OFFSET_DEG = float(os.getenv('GRIPPER_YAW_OFFSET_DEG', '0.0'))

# --- 비전 서비스(get_command) 관련 설정 ---
# 음성(get_keyword)과 달리 별도의 "시작 신호"가 없다. get_command.py 노드가 뜨는 순간
# 웹캠 스트리밍이 자동으로 시작되므로, 여기서도 노드 생성 직후부터 곧바로(그리고 계속)
# get_command 서비스를 폴링한다.
VISION_POLL_PERIOD_SEC = 0.2

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


class RobotController(Node):
    def __init__(self, mode: str = MODE):
        super().__init__("pick_and_place")
        self.mode = mode
        # [JSON 추가] 기존 task_json 노드가 Flask 전송을 담당한다.
        self.task_json = TaskJsonPublisher(self)
        

        # MultiThreadedExecutor 하에서 서비스 응답 콜백/타이머 콜백이 서로 블로킹 없이
        # 동시에 처리될 수 있도록 재진입 가능한 콜백 그룹을 사용한다.
        self.cb_group = ReentrantCallbackGroup()
        self.board_sync_client = self.create_client(
            SetBool, "/set_board_sync", callback_group=self.cb_group
        )

        self.gripper2cam_path = os.path.join(package_path,"resource","T_gripper2camera.npy")

        self.aruco_calculator = ArucoCalculator(node=self,
            get_current_posx_fn=get_current_posx,
            t_gripper_camera_path=self.gripper2cam_path,
            )

        self.get_logger().info("ArucoCalculator initialized")

        # hover 상태에서 SAM 기반 최적 파지 각도를 얻기 위한 서비스.
        # voice/vision 모드 모두 파지 직전에 이 서비스를 사용하므로 모드와
        # 무관하게 여기서 준비한다.
        self.get_grasp_plan_client = self.create_client(
            SrvGraspPlan, "/get_grasp_plan", callback_group=self.cb_group
        )

        self.get_logger().info(f"RobotController mode = '{self.mode}'")

        self.isBucket = False
        if self.mode == "voice":
            self._init_voice_services()
        else:
            self._init_vision_service()

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

    def _init_voice_services(self):
        """옵션 1(기본): get_keyword(음성) + get_position(depth) 서비스만 준비한다."""
        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position", callback_group=self.cb_group
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(
            Trigger, "/get_keyword", callback_group=self.cb_group
        )
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        self.ui_pub = self.create_publisher(String, "/ui/current_task", 10)
        self._publish_task(None, None)

    def _init_vision_service(self):
        """옵션 2: get_command(손동작 인식) 서비스만 준비하고 계속 폴링한다."""
        self.vision_client = self.create_client(
            Trigger, "get_command", callback_group=self.cb_group
        )
        while not self.vision_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_command (vision) service...")
        self.vision_request = Trigger.Request()

        # 아루코로 계산한 board_xyz_before(집는 위치)를 실제 detection 결과로
        # 보정하기 위한 서비스. 클래스 무관, 전체 detection 후보 중
        # 아루코 계산값과 가장 가까운 것을 robot_control 쪽에서 선택한다.
        self.get_all_positions_client = self.create_client(
            SrvAllPositions, "/get_all_positions", callback_group=self.cb_group
        )
        while not self.get_all_positions_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_all_positions service...")
        self.get_all_positions_request = SrvAllPositions.Request()

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

        self.get_logger().info(f"현황판 상태 변경: {'활성화' if enable else '비활성화'}")

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
        """Vision provides source/destination cells; detection resolves piece identity."""
        task_started = False
        try:
            result = future.result()
            if result is None or not result.success:
                return
            parts = result.message.split()
            if (len(parts) < 4 or parts[-1] not in ('release','bucket') or
                    (parts[-1] == 'release' and len(parts) < 7)):
                raise ValueError('Invalid vision command: ' + result.message)
            before = f'{parts[0]},{parts[2]}'
            after = f'{parts[4]},{parts[6]}' if parts[-1]=='release' else None
            self._record_task('start',before=before,after=after)
            task_started = True
            self.isBucket = parts[-1]=='bucket'
            # Preserve the requested cell XY. The old unconstrained nearest
            # detection refinement could move the reference to another piece.
            source = self.get_board_target_pos(before)
            destination = BUCKET_POS if self.isBucket else self.get_board_target_pos(after)
            if source is None or destination is None:
                raise RuntimeError('Source/destination board position unavailable')
            target_pos = [float(source[0]),float(source[1]),3.0,0.0,180.0,0.0]
            self.pick_and_place_target(target_pos,destination,grasp_selector='*')
            self.init_robot()
            self._record_task('completed' if rclpy.ok() else 'failed')
        except Exception as error:
            self.get_logger().error('Vision task failed: ' + str(error))
            if task_started:
                self._record_task('failed',error=str(error))
        finally:
            self.isBucket = False
            with self._vision_lock:
                self._vision_busy = False


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
    
    @staticmethod
    def _width_to_raw(opening_mm):
        """Physical inner gap -> RG register command, using measured pairs."""
        pairs = np.asarray(json.loads(os.getenv('RG2_WIDTH_CALIBRATION', '[]')), float)
        if pairs.ndim != 2 or pairs.shape[1] != 2 or len(pairs) < 2:
            raise ValueError('Set RG2_WIDTH_CALIBRATION to at least two [actual_gap_mm, raw] pairs')
        if (not np.all(np.isfinite(pairs)) or np.any(pairs < 0) or
                np.any(np.diff(pairs[:,0]) <= 0) or np.any(np.diff(pairs[:,1]) <= 0)):
            raise ValueError('Width calibration must increase in both actual gap and raw command')
        if np.any(pairs[:,1] > gripper.max_width):
            raise ValueError('Calibration exceeds gripper raw command limit')
        width = float(opening_mm)
        if not np.isfinite(width) or not pairs[0,0] <= width <= pairs[-1,0]:
            raise ValueError('Planned opening is outside the measured width calibration range')
        # Nearest register unit; do not assume raw/10 equals the physical gap.
        raw = int(round(float(np.interp(width, pairs[:,0], pairs[:,1]))))
        if not 0 <= raw <= gripper.max_width:
            raise ValueError('Invalid calibrated gripper command')
        return raw

    def get_grasp_plan(self, target, reference_pos):
        """Identify the physical target after camera motion and return pose + width."""
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
        future = self.get_grasp_plan_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=20.0) or not rclpy.ok():
            future.cancel()
            raise RuntimeError('Grasp plan timed out; no descent')
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError('Grasp plan failed: ' + (result.message if result else 'no response'))
        current_pose = np.asarray(get_current_posx()[0], float)
        R_before = self.get_robot_pose_matrix(*capture_pose)[:3,:3]
        R_after = self.get_robot_pose_matrix(*current_pose)[:3,:3]
        angle = math.degrees(math.acos(float(np.clip((np.trace(R_before.T @ R_after)-1)/2,-1,1))))
        if np.linalg.norm(current_pose[:3]-capture_pose[:3]) > 1 or angle > 1:
            raise RuntimeError('Camera moved during grasp inference; reacquire')
        camera_point = np.asarray(result.camera_position_mm, float)
        if camera_point.shape != (3,) or not np.all(np.isfinite(camera_point)):
            raise RuntimeError('Invalid SAM target position')
        base_xyz = T[:3,:3] @ camera_point + T[:3,3]
        if np.linalg.norm(base_xyz[:2]-np.asarray(reference_pos[:2])) > request.max_target_distance_mm:
            raise RuntimeError('Final SAM centre outside target gate')
        approach_base = T[:3,:3] @ np.asarray(result.approach_axis_camera, float)
        if not np.all(np.isfinite(approach_base)) or not np.allclose(approach_base,[0,0,-1],atol=0.02):
            raise RuntimeError('Planner approach differs from Base vertical')
        if not np.isfinite(result.clearance_mm) or result.clearance_mm <= 0:
            raise RuntimeError('Invalid clearance')
        rx,ry,rz = self._grasp_yaw_to_base_euler(result.closing_axis_camera, capture_pose)
        # Retain the already configured grasp Z (vision=3 mm). SAM adjusts XY/yaw only.
        pose = [float(base_xyz[0]),float(base_xyz[1]),float(reference_pos[2]),rx,ry,rz]
        raw = self._width_to_raw(result.grasp_width_mm)
        self.get_logger().info(
            f"Target={result.detected_name}, distance={result.target_distance_mm:.1f}mm, "
            f"opening={result.grasp_width_mm:.1f}mm -> raw={raw}, "
            f"clearance={result.clearance_mm:.1f}mm"
        )
        return pose, raw

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

        self.get_logger().info(f"Board coordinate: user=({row_user},{col_user}) -> internal=({row0},{col0})")
        board_xyz = self.aruco_calculator.get_base_point(row0,col0)

        if board_xyz is None:
            self.get_logger().warn("Aruco board BASE coordinate calculation failed")
            return None

        self.get_logger().info(f"Board {row_user}행 {col_user}열 BASE XYZ = {board_xyz}")
        return board_xyz

    def robot_control(self):
        target_list = []
        self.get_logger().info("call get_keyword service")
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
                # [JSON 추가] voice 말 이름과 도착 행·열 기록.
                self._record_task('start', piece=target, after=dest)
                try:
                    self._publish_task(target, dest)
                    board_xyz = self.get_board_target_pos(dest)
                    if board_xyz is None:
                        self.get_logger().warn(f"Invalid board target: {dest}")
                        self._record_task('failed', error=f'Board position unavailable: {dest}')
                        continue
                    target_pos = self.get_target_pos(target)
                    if target_pos is None:
                        self._record_task('failed', error=f'Piece not detected: {target}')
                        continue
                    self.pick_and_place_target(target_pos, board_xyz, grasp_selector=target)
                    self.init_robot()
                    self._record_task('completed' if rclpy.ok() else 'failed')
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
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        self._wait_for_future(get_position_future)
        if not rclpy.ok():
            return None

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            gripper2cam_path = os.path.join(
                package_path, "resource", "T_gripper2camera.npy"
            )
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, self.gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)

            target_pos = list(td_coord[:3]) + robot_posx[3:]
        return target_pos

    def refine_board_pos_with_detection(self, reference_xyz, max_dist=None):
        """아루코 기반 board_xyz(reference_xyz, BASE 프레임)와 가장 가까운
        실제 detection 결과를 찾아 BASE 프레임 좌표로 반환한다.
        클래스 무관, 화면에 보이는 모든 detection 후보 중 최근접을 사용한다.
        적절한 후보가 없거나 서비스 응답이 없으면 reference_xyz(아루코 계산값)를 그대로 반환한다.

        max_dist: None이 아니면, 최근접 후보와의 거리가 이 값(mm)을 넘을 때
                  오검출로 간주하고 아루코 계산값을 그대로 사용한다.
        """
        if reference_xyz is None:
            return None

        self.get_all_positions_request.min_score = 0.0
        future = self.get_all_positions_client.call_async(self.get_all_positions_request)
        self._wait_for_future(future, timeout_sec=5.0)
        if not rclpy.ok():
            return reference_xyz

        result = future.result()
        if result is None or len(result.x) == 0:
            self.get_logger().warn("get_all_positions: 후보 없음. 아루코 계산값 사용.")
            return reference_xyz

        robot_posx = get_current_posx()[0]
        ref = np.array(reference_xyz[:3], dtype=float)

        best_base_xyz = None
        best_dist = None
        for cam_x, cam_y, cam_z, score in zip(result.x, result.y, result.z, result.score):
            base_xyz = self.transform_to_base(
                [cam_x, cam_y, cam_z], self.gripper2cam_path, robot_posx
            )
            dist = float(np.linalg.norm(base_xyz - ref))
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_base_xyz = base_xyz

        if best_base_xyz is None or (max_dist is not None and best_dist > max_dist):
            self.get_logger().warn(
                f"get_all_positions: 유효 후보 없음(best_dist={best_dist}). 아루코 계산값 사용."
            )
            return reference_xyz

        self.get_logger().info(
            f"board_xyz_before 보정: 아루코={list(ref)} -> detection={list(best_base_xyz)} "
            f"(dist={best_dist:.2f}mm)"
        )
        return list(best_base_xyz)

    def init_robot(self):
        
        JReady = [-13, 21, 43, 0, 115.5, -13]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

        self.set_board_sync(True)

    def pick_and_place_target(self, target_pos, board_xyz, grasp_selector=None):
        # Validate calibration before moving: one observation (480 -> ~34 mm)
        # is insufficient to infer a complete physical-width mapping.
        pairs = json.loads(os.getenv('RG2_WIDTH_CALIBRATION','[]'))
        if not isinstance(pairs,list) or len(pairs)<2:
            raise ValueError('Configure RG2_WIDTH_CALIBRATION before robot motion')
        self._width_to_raw(float(pairs[0][0]))
        self.set_board_sync(False)
        lift_pos = list(target_pos[:2]) + [float(target_pos[2])+PLACE_LIFT,0.0,180.0,0.0]
        movel(lift_pos,vel=VELOCITY,acc=ACC)
        mwait()
        time.sleep(0.2)
        grasp_pos, raw = self.get_grasp_plan(grasp_selector,target_pos)
        # No fallback descent after a failed or ambiguous plan.
        rotated_hover = lift_pos[:3] + grasp_pos[3:]
        movel(rotated_hover,vel=VELOCITY,acc=ACC)
        mwait()
        gripper.move_gripper(raw)
        time.sleep(0.2)
        deadline = time.monotonic()+10.0
        while rclpy.ok() and gripper.get_status()[0]:
            if time.monotonic()>deadline:
                raise RuntimeError('Gripper opening timeout; no descent')
            time.sleep(0.05)
        if not rclpy.ok():
            raise RuntimeError('ROS stopped before descent')
        aligned_hover = grasp_pos[:2]+[lift_pos[2]]+grasp_pos[3:]
        movel(aligned_hover,vel=VELOCITY,acc=ACC)
        mwait()
        # Same XY and orientation for hover and grasp: pure vertical motion.
        movel(grasp_pos,vel=VELOCITY,acc=ACC)
        mwait()
        gripper.close_gripper()

        while rclpy.ok() and gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()

        # 실제로 집은 지점(grasp_pos)에서 바로 위로 들어올린다.
        # (파지 자세가 보정되었는데 여기서 옛 lift_pos를 쓰면 XY가 어긋난다.)
        retreat_pos = grasp_pos[:2] + [grasp_pos[2] + PLACE_LIFT] + grasp_pos[3:]
        movel(retreat_pos, vel=VELOCITY, acc=ACC)
        mwait()

        hover_pos = [float(board_xyz[0] + PLACE_X_OFFSET),float(board_xyz[1] + PLACE_Y_OFFSET), PLACE_LIFT,] + grasp_pos[3:]
        if self.isBucket :
            place_pos = [float(board_xyz[0] + PLACE_X_OFFSET),float(board_xyz[1] + PLACE_Y_OFFSET), BUCKET_POS[2], ] + grasp_pos[3:]
            self.isBucket = False
        else :
            place_pos = [float(board_xyz[0] + PLACE_X_OFFSET),float(board_xyz[1] + PLACE_Y_OFFSET), 4, ] + grasp_pos[3:]
        self.get_logger().info(f"Janggi place position: {place_pos}")

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
        # [JSON 추가] 종료 시 진행 중인 Task가 있으면 미완료로 기록한다.
        node._record_task('failed', error='Robot control stopped before completion')
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
