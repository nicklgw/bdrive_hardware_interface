bloom-generate rosdebian --os-name ubuntu --ros-distro humble
fakeroot debian/rules binary

/etc/ros/rosdep/sources.list.d/50-my-packages.list



colcon build
ros2 launch bdrive_hardware_interface real_robot_control.launch.py
