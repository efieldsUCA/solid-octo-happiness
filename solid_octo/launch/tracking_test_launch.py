from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. RealSense Camera Driver (The "Eyes")
    # Brings up the D455 and aligns depth to the color image so your tracking 
    # knows exactly how far away the colored pixels are.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'rgb_camera.profile': '640x480x30'
        }.items()
    )

    # 2. Static Transform (The "Neck")
    # Tells ROS exactly where the camera sits relative to the center of the robot.
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )

    # 3. Your Sorting Master Node (The "Brain")
    # This runs your custom OpenCV tracking code without the heavy SLAM overhead.
    sorting_master_node = Node(
        package='solid_octo', # Make sure this matches your actual package name!
        executable='sorting_master', 
        output='screen'
    )

    return LaunchDescription([
        realsense_launch,
        base_to_camera_tf,
        sorting_master_node
    ])
