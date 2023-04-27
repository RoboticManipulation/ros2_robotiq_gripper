# ros2_robotiq_gripper


In order to connect the gripper to the USB serial port, read and write permissions are required to function properly. To do this, add the username and execute the following command per user. Finally restart the system: 

sudo usermod -a -G dialout $USER

e.g.
Command has to be executed per user  

sudo usermod -a -G dialout kreis
sudo usermod -a -G dialout perurh