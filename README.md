# ros2_robotiq_gripper


In order to connect the gripper to the USB serial port, read and write permissions are required to function properly. To do this, add the username and execute the following command per user. Finally restart the system: 

```
sudo usermod -a -G dialout $USER

e.g.
Command has to be executed per user  

sudo usermod -a -G dialout kreis
sudo usermod -a -G dialout perurh
```

Clone repo:
```
git clone -b release/humble/serial git@github.com:RoboticManipulation/serial-release.git
git clone -b humble-test-setup git@github.com:RoboticManipulation/ros2_robotiq_gripper.git
```

Build:
```
colcon build --packages-skip robotiq_test_moveit
```

Start the controller:
```
ros2 launch robotiq_driver robotiq_control.launch.py
```

Check active controllers:
```
ros2 control list_controllers
```

Switch the controller:
```
ros2 control switch_controllers --activate robotiq_gripper_controller
```

Manually close:
```
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.8, max_effort: 0.01}}"
```

Manually open:
```
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.01}}"
```

