from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='voice',
        description='실행 모드: voice 또는 vision'
    )
    mode = LaunchConfiguration('mode')

    # 1. 도커 컨테이너 및 object_detection 노드 실행
    docker_cmd = ExecuteProcess(
        cmd=[
            'terminator',
            '-u', 
            '-x', 
            'bash', 
            '-c',
          'docker start yolo-detection && docker exec -it yolo-detection bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /ros2_ws/install/setup.bash && ros2 run object_detection object_detection"'],
        output='screen'
    )

    # 2. 메인 로봇 컨트롤 노드
    robot_control_node = Node(
        package='robot_control',
        executable='robot_control', # setup.py에 등록된 실행파일명에 맞게 수정
        arguments=['--mode', mode],
        output='screen'
    )

    # 3. Voice 모드: get_keyword 노드 실행
    voice_node = Node(
        package='voice_processing',
        executable='get_keyword',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'voice'"])),
        output='screen',
        prefix="terminator -u -x "  # <-- -u 옵션과 끝에 띄어쓰기 필수
    )

    # 4. Vision 모드: get_command 노드 실행
    vision_node = Node(
        package='hand_gesture_recognition',
        executable='get_command',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'vision'"])),
        output='screen',
        prefix="terminator -u -x "  # <-- -u 옵션과 끝에 띄어쓰기 필수
    )

    return LaunchDescription([
        mode_arg,
        docker_cmd,
        robot_control_node,
        voice_node,
        vision_node
    ])