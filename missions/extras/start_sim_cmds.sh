# Start px4
cd ~/PX4-Autopilot && make px4_sitl gz_x500_custom_2

# Start uxrce dds
micro-xrce-dds-agent udp4 -p 8888

# Start the ros_gz_bridge
ros2 run ros_gz_bridge parameter_bridge <topic_name>@<ros2_topic_type>@<gz_topic_type>

# Start the ros_gz_bridge with the config file
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/ros2_jazzy/src/px4_portfolio/missions/config/ros_gz_bridge_config.yaml

# Start the aruco detection node
ros2 run aruco_opencv aruco_tracker_autostart --ros-args -p cam_base_topic:=/camera_down/image

# Start needed ros2 topics
ros2 run missions controller
ros2 run missions line_detector

# Display an image topic
ros2 run image_tools showimage --ros-args -r image:=/camera_down/image
