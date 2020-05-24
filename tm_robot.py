#!/usr/bin/env python3
"""tm_robot.py: File content TM5 robot control class."""
__author__ = "Viet Toan"
__copyright__ = "Copyright 2019, ACM lab"
__license__ = "None"
__version__ = "1.0.0"
__email__ = "viettoan151@gmail.com"
__status__ = "Development"

import socket
import struct
import time
import os
import numpy as np

from tm_communication import TmCommunication, CommRecvError
from tm_robot_state_rt import TmRobotStateRT, RobotError

TMROBOT_DEBUG = False  # use TM5 robot and do debug (with or without robot)

class TM5_Robot(object):
    def __init__(self,robot_tcp_ip='192.168.0.10', robot_tcp_port=6188,
                 camera_tcp_ip='', camera_tcp_port=60000, is_sim = False):
        self.is_sim = is_sim
        # If in simulation...
        if self.is_sim:
            print('Simulation has not been finished yet')
        # If in real-settings...
        else:
            # Connect to robot client
            self.robot_tcp_ip = robot_tcp_ip
            self.robot_tcp_port = robot_tcp_port
            self.camera_tcp_ip = camera_tcp_ip
            self.camera_tcp_port = camera_tcp_port

            # Default home joint configuration
            self.home_joint_config = [90.0, 0.0, 90.0, 0.0, 90.0, 0.0]  # q[1..6]
            self.zero_gripper_offset = [0.0, 0.0, 0.12]  # XYZ in meter
            self.zero_tool_orientation = [180.0, 0.0, 0.0]  # ABC in degree

            # Default joint speed configuration
            self.joint_acc = 8  # Safe: 1.4
            self.joint_vel = 3  # Safe: 1.05

            # Joint tolerance for blocking calls
            self.joint_tolerance = 0.01

            # Default tool speed configuration
            self.tool_acc = 1.2  # Safe: 0.5
            self.tool_vel = 0.25  # Safe: 0.2

            # Tool pose tolerance for blocking calls
            self.tool_pose_tolerance = [0.002, 0.002, 0.002, 0.01, 0.01, 0.01]

            if TMROBOT_DEBUG:
                print('This is debug tool action')
            else:
                self.RobotComm = TmCommunication()

            # Move robot to home pose
            self.close_gripper()
            self.go_home()

    def close_gripper(self):
        '''
        close_gripper
        :return: True/False gripper close status
        :NOTE: please don't use this return to check grasp object
        '''
        if self.is_sim:
            gripper_fully_closed = True
        elif TMROBOT_DEBUG:
            print('Tool debug close gripper')
            gripper_fully_closed = True
        else:
            cmd = 'digop 8 1 0'
            self.RobotComm.sendCommandMsg(cmd)
            time.sleep(0.5)
            temp = self.RobotComm.stateRT.st_ToolDigitalInput
            gripper_fully_closed = (temp == self.RobotComm.stateRT.GRIPPER_FULL_CLOSE)
        return gripper_fully_closed

    def open_gripper(self):
        '''
        open_gripper
        :return: NA
        '''
        if self.is_sim:
            self.gripper_handler.open_gripper(self.gripper_typer)

        elif TMROBOT_DEBUG:
            print('Tool debug open gripper')
        else:
            cmd = 'digop 8 0 0'
            self.RobotComm.sendCommandMsg(cmd)
            time.sleep(0.5)

    def move_to(self, tool_position, tool_orientation):
        '''
        move_to controls the robot by tool Cartesian coordinate, and angles
        :param tool_position: [x,y,z]tool position in robot's Cartesian coordinate
        :param tool_orientation: [alpha, beta, gama]tool orientation angles in Degree
        :return: NA
        :exception: RobotError when the moving have any error
        '''
        if self.is_sim:
            print('This part has not been implemented yet')
        elif TMROBOT_DEBUG:
            print('Debug tool move to:{}, angle:{}'.format(tool_position, tool_orientation))
        else:
            tcp_command = "movl 0 %.4f %.4f %.4f %.4f %.4f %.4f %.1f" % (
                            tool_position[0], tool_position[1], tool_position[2],
                            tool_orientation[0], tool_orientation[1], tool_orientation[2], 0)
            try:
                self.RobotComm.sendCommandMsg(tcp_command)
            except RobotError as exc:
                print('Error in move robot')
                raise exc

    def move_joints(self, joints_configuration):
        '''
        move_joints moves the robot by control 6 joints angle
        :param joints_configuration: angle of 6 joints in Degree
        :return: None
        :exception: RobotError when the moving have any error
        '''
        if self.is_sim:
            print('This part has not been implemented yet')
        elif TMROBOT_DEBUG:
            print('Move joints:{}'.format(joints_configuration))
        else:
            tcp_command = "movj 0"
            for joint_idx in range(0, 6):
                tcp_command = tcp_command + (" %.4f" % joints_configuration[joint_idx])
            tcp_command = tcp_command + " 0"
            try:
                self.RobotComm.sendCommandMsg(tcp_command)
            except RobotError as exc:
                print('Error in move joints robot')
                raise exc

    def go_home(self):
        '''
        go_home moves robot to default position by joints control
        :return: NA
        :exception: RobotError when the moving have any error
        '''
        if self.is_sim:
            print('This part has not been implemented yet')
        elif TMROBOT_DEBUG:
            print('Tool debug go home')
        else:
            try:
                self.move_joints(self.home_joint_config)
            except RobotError as exc:
                print('Error in robot go home')
                raise exc

    def clear_error(self, exc):
        '''
        clear_error does the default action cmdc0 and go home
        :param exc: exception variable
        :return: no
        '''
        print('Clear error:{}{}'.format(exc.expression[0],exc.expression[1]))
        self.RobotComm.sendCommandMsg('cmdc0')
        self.go_home()

    def check_grasp(self):
        '''
        Check Grasp status
        :return: True/False status of have object in gripper
        NOTE: must be preceded by close_gripper()
        '''
        rtn = False
        if TMROBOT_DEBUG:
            print('Tool debug check grasp OK')
            rtn = True
        else:
            try:
                self.RobotComm.readRobotState()
                temp = self.RobotComm.stateRT.st_ToolDigitalInput
                rtn = (temp == self.RobotComm.stateRT.GRIPPER_HAS_OBJ)
            except CommRecvError as exc:
                rtn = False
        return rtn

    def free_servo_enable(self, enable=True):
        """
        enable tp free robot servo
        NOTE: Please check LED turn to Green light if enable = True
        :param enable: enable or disable free mode of servo
        :return: True when parameter 'enable' and final teach mode are same
        """
        rtn = False
        tcp_command = 'cmd50 0 ' + '1' if enable else '0'
        if self.is_sim:
            print('This part has not been implemented yet')
        elif TMROBOT_DEBUG:
            print('Send free {} command:{}'.format('enable' if enable else 'disable',tcp_command))
        else:
            try:
                # send blocking command to get robot state after command finished
                self.RobotComm.sendCommandMsg(tcp_command)
                # get back teach mode state
                tmp = self.RobotComm.stateRT.st_TeachMode
                # check teach mode and parameter
                rtn = (enable == bool(tmp))
            except RobotError as exc:
                print('Error in enable free robot')
                rtn = False
        return rtn

    def test_robot(self):
        while (True):
            lett = input('Input your choise:')
            if (lett == 's'):
                print('Read robot state')
                if(self.is_sim):
                    print('Simulation read')
                else:
                    self.RobotComm.readRobotState()
                    print('Robot:{}'.format(self.RobotComm.stateRT.st_ErrorCode_0))
                    print('Robot joint position:{}', format(self.RobotComm.stateRT.st_ActJointsPos))
                    print('Tool cartesian data:{}', format(self.RobotComm.stateRT.st_ActToolPos))
            elif (lett == 'h'):
                print('Robot go home')
                self.go_home()
            elif (lett =='g'):
                print('Toggle gripper')
                self.RobotComm.readRobotState()
                temp = self.RobotComm.stateRT.st_ToolDigitalInput
                if(temp == self.RobotComm.stateRT.GRIPPER_FULL_OPEN):
                    print('Close gripper')
                    self.close_gripper()
                elif(temp == self.RobotComm.stateRT.GRIPPER_FULL_CLOSE):
                    print('Open gripper')
                    self.open_gripper()
                else:
                    print('Gripper has object, open gripper')
                    self.open_gripper()
            elif (lett == 'c'):
                cmd = input('Please give command:')
                try:
                    self.RobotComm.sendCommandMsg(cmd)
                except RobotError as exc:
                    print('Error in command')
                    self.clear_error(exc)

            elif (lett == ('q' or 'Q')):
                break
        return 0


