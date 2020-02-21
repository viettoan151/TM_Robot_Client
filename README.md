# TM-Robot-Client
The client control for Techman TM5 robot arm
 Modules are shown in level top-down
robot_test.py
**tm_robot.py**
Implementation of some basic robot actions
- Open/Close gripper
- Move to
- Move joints
- Go home
- Check grapping status

**tm_robot_state_rt.py**
Helps to parse TCP package from robot, and updates robot status

**tm_communication.py**
Handle TCP/IP connection between host and robot
It includes sending command, read back robot status
