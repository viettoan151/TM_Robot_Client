# TM-Robot-Client
The client control for Techman TM5 robot arm.
Modules are shown in level top-down

**robot_test.py**

**tm_robot.py:**
Implementation of some basic robot actions under class TM5_Robot:
- Open/Close gripper
- Move to
- Move joints
- Go home
- Check grasping status
- Enable free servos

**tm_robot_state_rt.py:**
Class TmRobotStateRT has robot state and method to parse TCP packages from robot

**tm_communication.py:**
Handle TCP/IP connection between host and robot.
It includes sending command, read back robot status
All robot state in TmRobotStateRT to be updated after called to readRobotState.
