bloom-generate rosdebian --os-name ubuntu --ros-distro humble
fakeroot debian/rules binary

/etc/ros/rosdep/sources.list.d/50-my-packages.list

colcon build
export LD_LIBRARY_PATH=/usr/local/bzl_robot/lib:$LD_LIBRARY_PATH # 实体车上运行时, 设置动态库环境变量
ros2 launch bdrive_hardware_interface real_robot_control.launch.py

ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray '{"data":[1.0]}'
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray '{"data":[1.0]}'

位置模式速度模式相互切换
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController '{"activate_controllers":["forward_position_controller"], "deactivate_controllers":["forward_velocity_controller"],"strictness":1,"activate_asap":false}'
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController '{"activate_controllers":["forward_velocity_controller"], "deactivate_controllers":["forward_position_controller"],"strictness":1,"activate_asap":false}'
ros2 control switch_controllers --activate forward_velocity_controller --deactivate forward_position_controller
ros2 control switch_controllers --activate forward_position_controller --deactivate forward_velocity_controller
