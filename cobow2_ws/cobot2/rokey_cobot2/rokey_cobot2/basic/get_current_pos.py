

import rclpy
from rclpy.node import Node

import tkinter as tk
from tkinter import StringVar
import threading

import time
import DR_init

from dsr_msgs2.srv import SetRobotMode

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
ON, OFF = 1, 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def copy_to_clipboard(root, text_box):
    text = text_box.get().strip()
    root.clipboard_clear()
    root.clipboard_append(text)
    root.update()


def create_entries(root, default_value, row, col):
    entry_var = StringVar()
    entry_var.set(str(round(default_value, 3)))
    entry = tk.Entry(root, textvariable=entry_var, width=50)
    entry.grid(row=row, column=col, padx=10, pady=5)
    return entry_var


class ServiceClinetNode(Node):
    def __init__(self):
        super().__init__("service_client_node")
        # dsr_controller2 는 자기 노드 이름을 서비스 앞에 붙여서 연다.
        # 이 접두어가 빠지면 wait_for_service 가 영원히 끝나지 않는다.
        self.cli = self.create_client(
            SetRobotMode, f"/{ROBOT_ID}/dsr_controller2/system/set_robot_mode"
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("Waiting for service...")
            pass

    def send_request(self, mode=0):
        request = SetRobotMode.Request()
        request.robot_mode = mode
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def ros_thread(text_var1, text_var2):
    try:
        from DSR_ROBOT2 import get_current_posx, get_current_posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    while rclpy.ok():
        try:
            posx_res = get_current_posx()
            if posx_res is not None:
                data_x = [round(d, 3) for d in posx_res[0]]
                text_var1.set(f"posx({data_x})")

            posj_res = get_current_posj()
            if posj_res is not None:
                data_j = [round(d, 3) for d in posj_res]
                text_var2.set(f"posj({data_j})")

            time.sleep(0.1)

        except Exception:
            time.sleep(0.5)
            continue

    rclpy.shutdown()


def main():
    root = tk.Tk()
    tk.Label(root, text="current_posx:").grid(row=0, column=0)
    text_var1 = create_entries(root, 0.0, 0, 1)
    tk.Button(root, text="copy", command=lambda: copy_to_clipboard(root, text_var1)).grid(
        row=0, column=3, padx=2, pady=5
    )

    tk.Label(root, text="joint_state:").grid(row=1, column=0)
    text_var2 = create_entries(root, 0.0, 1, 1)
    tk.Button(root, text="copy", command=lambda: copy_to_clipboard(root, text_var2)).grid(
        row=1, column=3, padx=2, pady=5
    )

    print("Service Start")
    rclpy.init()

    dsr_node = rclpy.create_node("dsr_global_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    client_node = ServiceClinetNode()
    response = client_node.send_request(0)
    client_node.get_logger().info(f"results: {response}")

    ros = threading.Thread(target=ros_thread, args=(text_var1, text_var2))
    ros.start()

    root.mainloop()


if __name__ == "__main__":
    main()