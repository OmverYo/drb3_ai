import os
import time
import sys
import json
import argparse
import threading
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import DR_init

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG
from robot_control.aruco_calculator import ArucoCalculator

package_path = get_package_share_directory("robot_control")

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
BUCKET_POS = [4.00, 38.00, 64.00, -0.1, 78.0, 4]
JHOME_POS = [0, -30, 90, 0, 90, 0]
PLACE_LIFT = 250.0
PLACE_Z_OFFSET = 50.0
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -35.0
MIN_DEPTH = 2.0

# --- 비전 서비스(get_command) 관련 설정 ---
# 음성(get_keyword)과 달리 별도의 "시작 신호"가 없다. get_command.py 노드가 뜨는 순간
# 웹캠 스트리밍이 자동으로 시작되므로, 여기서도 노드 생성 직후부터 곧바로(그리고 계속)
# get_command 서비스를 폴링한다.
VISION_POLL_PERIOD_SEC = 0.2
Z_BUMP_MM = 10.0  # 1cm
# pick-and-place용 VELOCITY/ACC(60,60)를 그대로 쓰면 bump 왕복이 너무 짧게 끝나서
# "이동 중엔 비전 요청 안 함" 동작이 눈에 잘 안 띈다. bump 전용으로 훨씬 느린 속도를 쓴다.
VISION_BUMP_VELOCITY = 5
VISION_BUMP_ACC = 5

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
        self.init_robot()

        # MultiThreadedExecutor 하에서 서비스 응답 콜백/타이머 콜백이 서로 블로킹 없이
        # 동시에 처리될 수 있도록 재진입 가능한 콜백 그룹을 사용한다.
        self.cb_group = ReentrantCallbackGroup()

        self.gripper2cam_path = os.path.join(package_path,"resource","T_gripper2camera.npy")

        self.aruco_calculator = ArucoCalculator(node=self,
            get_current_posx_fn=get_current_posx,
            t_gripper_camera_path=self.gripper2cam_path,
            )

        self.get_logger().info("ArucoCalculator initialized")

        self.get_logger().info(f"RobotController mode = '{self.mode}'")

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
        """
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info(f"vision command: {result.message}")
                # result.message example: "1 , 3 grap 3 , 4 release"
                text_split = result.message.split(' ') 
                board_lc_before = f'{text_split[0]},{text_split[2]}'
                board_lc_after = f'{text_split[4]},{text_split[6]}'

                board_xyz_before = self.get_board_target_pos(board_lc_before)
                before_pos = [np.float64(board_xyz_before[0]),np.float64(board_xyz_before[1]),np.float64(board_xyz_before[2])] + [0.0, 180.0, 0.0]

                board_xyz_after = self.get_board_target_pos(board_lc_after)
                #after 값은 pick_and_place_target() 내부에서 after_pos 화
            else:
                reason = result.message if result is not None else "no_response"
                self.get_logger().info(f"vision service 응답 실패/대기중: {reason}")
        except Exception as e:
            self.get_logger().error(f"vision 서비스 응답 처리 실패: {e}")

        try:
            #self._bump_z()
            self.pick_and_place_target(before_pos, board_xyz_after)
            self.init_robot()
        except Exception as e:
            self.get_logger().error(f"vision 이동 실패: {e}")
        finally:
            # 이동이 끝난 뒤에야 다음 요청을 허용 (조건 3)
            with self._vision_lock:
                self._vision_busy = False

    def _bump_z(self):
        """현재 위치에서 z축으로 1cm 올라갔다가 원래 위치로 되돌아온다.

        pick-and-place 동작(VELOCITY/ACC=60,60)보다 훨씬 느린 속도로 움직여서,
        이 이동이 진행되는 동안 비전 서비스 요청이 안 나가는 것(조건 3)을 눈으로
        확인할 수 있을 만큼 충분히 오래 걸리게 한다.
        """
        current_pos = get_current_posx()[0]
        up_pos = list(current_pos[:2]) + [current_pos[2] + Z_BUMP_MM] + list(current_pos[3:])
        movel(up_pos, vel=VISION_BUMP_VELOCITY, acc=VISION_BUMP_ACC)
        mwait()
        movel(current_pos, vel=VISION_BUMP_VELOCITY, acc=VISION_BUMP_ACC)
        mwait()

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
                self._publish_task(target, dest)
                board_xyz = self.get_board_target_pos(dest)
                if board_xyz is None:
                    self.get_logger().warn(f"Invalid board target: {dest}")
                    continue
                target_pos = self.get_target_pos(target)
                if target_pos is None:
                    continue
                self.pick_and_place_target(target_pos, board_xyz)
                self.init_robot()

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

    def init_robot(self):
        JReady = [-18, 2, 66, 0, 111, -17.6]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos, board_xyz):
        self.get_logger().info(f"temp ljs type : {type(target_pos)}")
        self.get_logger().info(f"temp ljs len : {len(target_pos)}")
        self.get_logger().info(f"temp ljs type : {type(target_pos[0])} {type(target_pos [1])} {type(target_pos[2])} {type(target_pos[3])} {type(target_pos[4])}")
        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper()

        while rclpy.ok() and gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()

        lift_pos = target_pos[:2] + [target_pos[2] + PLACE_LIFT] + target_pos[3:]
        movel(lift_pos, vel=VELOCITY, acc=ACC)
        mwait()

        place_pos = [float(board_xyz[0]),float(board_xyz[1]),float(board_xyz[2] + PLACE_Z_OFFSET), ] + target_pos[3:]
        self.get_logger().info(f"Janggi place position: {place_pos}")
        movel(place_pos, vel=VELOCITY, acc=ACC)
        mwait()

        gripper.open_gripper()
        while rclpy.ok() and gripper.get_status()[0]:
            time.sleep(0.5)


def main(args=None):
    node = RobotController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
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
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
