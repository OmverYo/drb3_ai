import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    task_json_node = Node(
            package='robot_control',
            executable='task_json',
            output='screen',
            #prefix="terminator -u -x "  # <-- -u 옵션과 끝에 띄어쓰기 필수
        )
    # 1. 로봇 연결 (메인 터미널 실행)
    roboton_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('m0609_rg2_bringup'), 
                'launch', 
                'bringup.launch.py'
            )
        ),
        launch_arguments={
            'mode': 'real',
            'host': '192.168.1.100'
        }.items(),
        
    )

    # 2. Realsense 카메라 (새 창)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='/',
        name='camera',
        # output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'depth_module.depth_profile': '848x480x30',
            'rgb_camera.color_profile': '1280x720x30',
            'align_depth.enable': True,
            'enable_rgbd': True,
            'enable_sync': True,
            'pointcloud.enable': True,
            'pointcloud.stream_filter': 2,
            'enable_accel': False,
            'enable_gyro': False,
            'initial_reset': True
        }],
        #prefix="terminator -u -x "  # <-- -u 옵션과 끝에 띄어쓰기 필수
    )

    # 3. Task JSON 노드 (새 창)
    

    return LaunchDescription([
        roboton_launch,
        realsense_node,
        task_json_node
    ])