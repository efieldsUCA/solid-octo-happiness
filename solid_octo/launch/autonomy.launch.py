import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # --- CONFIGURATION ---
    # Where to save the RTAB-Map database
    db_path = os.path.expanduser('~/.ros/rtabmap.db')

    # --- 1. ROBOT BASE (The Wheels) ---
    # REPLACE this with your actual robot launch file if you have one.
    # For now, we publish a static TF so the map works even without wheels moving.
    # If your robot driver publishes /odom, you can remove the 'static_transform_publisher' below.
    octo_launch = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # --- 2. TF: Connect Camera to Robot ---
    # Mounting position: 10cm forward (x=0.1), 20cm up (z=0.2)
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_connector',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )

    # --- 3. THE CAMERA (RealSense) ---
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2' # Linear interpolation
        }.items()
    )

    # --- 4. THE FAKE LASER (Depth -> LaserScan) ---
    # This turns the 3D depth camera into a 2D laser scanner for Nav2
    depth_to_laser_cmd = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan')
        ],
        parameters=[{
            'output_frame': 'camera_link',
            'range_min': 0.3,
            'range_max': 4.0, # Limit to 4m to avoid noise
            'scan_height': 10 # Check 10 rows of pixels to be safe
        }]
    )

    # --- 5. THE MAPPER (RTAB-Map SLAM) ---
    rtabmap_launch = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'database_path': db_path,
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_scan': True, # We use our fake laser scan!
            'approx_sync': True,
            'Reg/Strategy': '1',    # 1=ICP (Laser scan matching)
            'Grid/RayTracing': 'true',
            'RGBD/NeighborLinkRefining': 'true',
            'use_sim_time': False
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odom') # Assuming your robot or visual odometry publishes this
        ],
        arguments=['--delete_db_on_start'] # Clears the map every restart. Remove this to keep memory!
    )

    # --- 6. THE NAVIGATOR (Nav2) ---
    # This launches the Path Planner and Controller
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        octo_launch,
        tf_base_to_camera,
        realsense_launch,
        depth_to_laser_cmd,
        TimerAction(period=5.0, actions=[rtabmap_launch]), # Wait 5s for camera to warm up
        TimerAction(period=8.0, actions=[nav2_launch])     # Wait 8s for map to start
    ])
