# solid-octo-happiness
Project that uses SLAM navigation and RP LiDAR technology to create an autonomous robot that has clamping mechanism for gathering objects. Update: Using Realsense D455 camera instead of RP LiDar for visual SLAM.

### Initializing Robot Control Nodes:
To get the robot started, boot up the Pi 5 and ensure all electrical components are connected (Pico Microcontroller, Motor drivers, stepper motors, etc.). Then follow the following node sequence in seperate terminals to launch the controller operation for debugging:
### Terminal 1:  cd ~/seniordesign_ws -> source install/local_setup.bash -> ros2 launch solid_octo octo_launch.py 

### Terminal 2: ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true unite_imu_method:=2 enable_gyro:=true enable_accel:=true init_reset:=true
### Terminal 3: source /opt/ros/jazzy/setup.bash -> ros2 launch imu_filter_madgwick imu_filter.launch.py 
### Terminal 4: ros2 launch rtabmap_launch rtabmap.launch.py rtabmap_args:="--delete_db_on_start" rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/aligned_depth_to_color/image_raw camera_info_topic:=/camera/camera/color/camera_info frame_id:=camera_link imu_topic:=/imu_fused rviz:=true rtabmap_viz:=false
