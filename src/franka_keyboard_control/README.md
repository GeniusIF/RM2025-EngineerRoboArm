```bash
colcon build --packages-up-to franka_keyboard_control
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
source install/setup.zsh
ros2 launch franka_keyboard_control franka_interface.launch.py
ros2 run franka_keyboard_control rm_servo_keyboard_input
```

```bash
sudo chmod 777 ./script/create_udev_rules.sh
./script/create_udev_rules.sh
```