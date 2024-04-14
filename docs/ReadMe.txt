bloom-generate rosdebian --os-name ubuntu --ros-distro humble
fakeroot debian/rules binary

/etc/ros/rosdep/sources.list.d/50-my-packages.list

colcon build
export LD_LIBRARY_PATH=/usr/local/bzl_robot/lib:$LD_LIBRARY_PATH # 实体车上运行时, 设置动态库环境变量
ros2 launch bdrive_hardware_interface real_robot_control.launch.py

ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray '{"data":[1.0]}'
