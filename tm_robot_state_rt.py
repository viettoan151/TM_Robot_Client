#!/usr/bin/env python3
"""tm_robot_state.py: File content TM5 robot state class."""
__author__ = "Viet Toan"
__copyright__ = "Copyright 2019, ACM lab"
__license__ = "None"
__version__ = "1.0.0"
__email__ = "viettoan151@gmail.com"
__status__ = "Development"

import struct
import numpy as np

class TmRobotStateRT(object):
    CORRECT_RECVDATA_SIZE = 372
    GRIPPER_FULL_OPEN = 0x01
    GRIPPER_FULL_CLOSE = 0x02
    GRIPPER_HAS_OBJ = 0x04
    def __init__(self, x):
        self.i=0
        self.data_length=int(0)     # data length receive from TCP/IP, expected is CORRECT_RECVDATA_SIZE
        self.controller_time_ms= 0
        self.rb_CommandDone = True  # robot done the execution, Gripper does not use this
        self.st_SafetyMode = 0
        self.st_ControlMode = 0
        self.st_TeachMode = 0
        self.st_ReqQnCmdCount = 0
        self.st_ActJointsPos=[]
        self.st_TargetJointPos = []
        self.st_CmdJointsPos=[]
        self.st_ActToolPos=[]
        self.st_ToolDigitalInput = 0
        self.st_BuffEmptyFlag = 0
        self.st_ErrorCode_0 = 0
        self.st_ErrorCode_1 = 0
    
    def parse_tcp_state_data(self, data_bytes):
        '''
        parse_tcp_state_data parse received tcp data
            :argument
                data_bytes:
            :return
                1: correct data length
            :exception
                RobotDataError : when receive incorrect data length
                RobotError : when have error in robot which reported
        '''
        # Read package header
        #package length is bye 0, size 2, unsigned short, bigendian
        temp= struct.unpack(">H", data_bytes[0:2])[0]
        if temp != self.CORRECT_RECVDATA_SIZE:
            self.data_length = 0
            raise RobotDataError(-1,'ERROR: Robot receive incorrect data length!')
        self.data_length = temp
        #controller time is bye 4, size 8, unsigned long long, little endian
        self.controller_time_ms = struct.unpack("<Q", data_bytes[4:12])[0]
        # Parse basic information
        self.st_SafetyMode = struct.unpack("<B",data_bytes[359:360])[0]
        self.st_ControlMode = struct.unpack("<B",data_bytes[360:361])[0]
        self.st_TeachMode = struct.unpack("<B",data_bytes[361:362])[0]
        self.st_ReqQnCmdCount = struct.unpack("<I",data_bytes[362:366])[0]
        self.st_BuffEmptyFlag = struct.unpack("<I",data_bytes[366:370])[0]
        self.st_ErrorCode_0 = struct.unpack("<B",data_bytes[370:371])[0]
        self.st_ErrorCode_1 = struct.unpack("<B",data_bytes[371:372])[0]

        #Parse joints information
        byte_idx = 12
        self.st_ActJointsPos = [0,0,0,0,0,0]
        self.st_TargetJointPos = [0,0,0,0,0,0]
        for joint_idx in range(6):
            self.st_ActJointsPos[joint_idx] = struct.unpack('<f', data_bytes[(byte_idx+0):(byte_idx+4)])[0]
            self.st_TargetJointPos[joint_idx] = struct.unpack('<f', data_bytes[(byte_idx+24):(byte_idx+28)])[0]
            byte_idx += 4

        #parse_cartesian_info
        byte_idx = 204
        self.st_ActToolPos = [0,0,0,0,0,0]
        for pose_value_idx in range(6):
            self.st_ActToolPos[pose_value_idx] = struct.unpack('<f', data_bytes[(byte_idx+0):(byte_idx+4)])[0]
            byte_idx += 4

        #parse_tool_data
        byte_idx = 356
        self.st_ToolDigitalInput = struct.unpack('<B', data_bytes[(byte_idx+0):(byte_idx+1)])[0]

        #update command_done flag
        self.rb_CommandDone = True if(self.st_BuffEmptyFlag == 0x000000FC) else False
        #raise error
        if(self.st_ErrorCode_0 != 0):
            raise RobotError((self.st_ErrorCode_0,self.st_ErrorCode_1),
                             'ERROR: Robot get error, please check error code!')
        return 1

class RobotError(Exception):
    def __init__(self, expression, message):
        self.expression = expression
        self.message = message

class RobotDataError(Exception):
    def __init__(self, expression, message):
        self.expression = expression
        self.message = message
