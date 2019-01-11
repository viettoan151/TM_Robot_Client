# -*- coding: utf-8 -*-
import struct

class TmRobotStateRT(object):
    def __init__(self, x):
        self.i=0
        self.data_length=int(0)
        self.controller_time_ms= 0
        self.command_done = True
    def getData(self):
        return 0
    
    def parse_tcp_state_data(self, data_bytes, subpackage='None'):
        '''
        parse recived tcp data:
            input: 
                data_bytes:
                subpackage:
                    - 'None'
                    - 'robot_state'
                    - 'joint_data'
                    - 'cartesian_info'
                    - 'tool_data'
            return:
                a dictionary of value
            
        '''
        # Read package header
        #package length is bye 0, size 2, unsigned short, bigendian
        self.data_length = struct.unpack(">H", data_bytes[0:2])[0]
        #controller time is bye 4, size 8, unsigned long long, little endian
        self.controller_time_ms = struct.unpack("<Q", data_bytes[4:12])[0]
        #print("Data length:{}, Length:{}".format(len(data_bytes),self.data_length))
        # Parse sub-packages
        if subpackage == 'None':
            return 1
        def parse_robot_state(data_bytes):
            safety_mode = struct.unpack("<B",data_bytes[359:360])[0]
            control_mode = struct.unpack("<B",data_bytes[360:361])[0]
            res_que_cmd_count = struct.unpack("<I",data_bytes[362:366])[0]
            buf_empty_flag = struct.unpack("<I",data_bytes[366:370])[0]
            #update command_done flag
            self.command_done = True if(buf_empty_flag == 0x000000FC) else False
                
            error_code_0 = struct.unpack("<B",data_bytes[370:371])[0]
            error_code_1 = struct.unpack("<B",data_bytes[371:372])[0]
            return {'safety_mode':safety_mode, 'control_mode':control_mode,
                    'res_que_cmd_count':res_que_cmd_count, 'buf_empty_flag':buf_empty_flag,
                    'command_done':self.command_done,
                    'error_code_0':error_code_0, 'error_code_1':error_code_1}
    
        def parse_joint_data(data_bytes):
            byte_idx = 12
            actual_joint_positions = [0,0,0,0,0,0]
            target_joint_positions = [0,0,0,0,0,0]
            for joint_idx in range(6):
                actual_joint_positions[joint_idx] = struct.unpack('<f', data_bytes[(byte_idx+0):(byte_idx+4)])[0]
                target_joint_positions[joint_idx] = struct.unpack('<f', data_bytes[(byte_idx+24):(byte_idx+28)])[0]
                byte_idx += 4
            return actual_joint_positions
    
        def parse_cartesian_info(data_bytes):
            byte_idx = 204
            actual_tool_pose = [0,0,0,0,0,0]
            for pose_value_idx in range(6):
                actual_tool_pose[pose_value_idx] = struct.unpack('<f', data_bytes[(byte_idx+0):(byte_idx+4)])[0]
                byte_idx += 4
            return actual_tool_pose
    
        def parse_tool_data(data_bytes):
            byte_idx = 356
            tool_digital_input = struct.unpack('<B', data_bytes[(byte_idx+0):(byte_idx+1)])[0]
            return tool_digital_input
    
        parse_functions = {'robot_state' : parse_robot_state,'joint_data' : parse_joint_data, 'cartesian_info' : parse_cartesian_info, 'tool_data' : parse_tool_data}
        return parse_functions[subpackage](data_bytes)