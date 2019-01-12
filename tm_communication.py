#!/usr/bin/env python3
"""tm_communication.py: File content TM5 robot control client TCP/IP class."""
__author__ = "Viet Toan"
__copyright__ = "Copyright 2019, ACM lab"
__license__ = "None"
__version__ = "1.0.0"
__email__ = "viettoan151@gmail.com"
__status__ = "Development"

from tm_robot_state_rt import TmRobotStateRT, RobotError, RobotDataError
import socket
import threading
import struct
import time

COMMDEBUG = False
COMMRUNMODE = True

class TmCommunication(object):
    DEFAULT_TIMEOUT = 5000
    DEFAULT_TIMEVAL = 500
    MAX_NUM_OF_PORT = 2
    MAX_RECVDATA_SIZE = 1024
    CORRECT_RECVDATA_SIZE = 372
    MAX_SENDDATA_SIZE = 512
    SB_SIZE = 2048
    socket_lock = threading.Condition()
    def __init__(self, data_condv_rt=None, ip='192.168.0.10', port=6188, timeout_ms=5000, timeval_ms=500):
        print("[INFO]TM_COM: TMCommunication Constructor")
        if COMMDEBUG:
            print("[DEBUG]TM_COM: This is DEBUG MODE of communication module!!!!")
        self.sockfd=None
        self.stateRT = TmRobotStateRT(data_condv_rt)
        self.sb_H = 0
        self.sb_T = 0
        self.server_ip=ip
        self.server_port=port
        self.connect_timeout_s = timeout_ms / 1000;
        self.thread_timeval_s  = timeval_ms / 1000;
        self.data_bytes = bytearray()
        self.data_ready = False
        print("[DEBUG]TM_COM: TMCommunication Construction DONE")

    def connect(self):
        '''
        connect Connect to host in configured IP and Port
        :return: success
        :exception socket.error: can't create connection to host
        '''
        #acquire the lock
        self.socket_lock.acquire()
        #configure the socket
        self.sockfd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sockfd.settimeout(self.connect_timeout_s)
        try:
            rtn = self.sockfd.connect((self.server_ip,self.server_port))
            return rtn
        except socket.error as exc:
            print('[ERROR]TM_COM:Can\'t create connection to robot: %s' % exc)
            raise exc
        
    def disconnect(self):
        '''
        disconnect
        :return: NA
        '''
        self.sockfd.close()
        self.socket_lock.release()
        if COMMDEBUG:
            print("[INFO]TM_COM:Connection to TM robot CLOSED")

    def readRobotState(self):
        '''
        readRobotSate open TCP/IP connect to host and receive host data
        :return: 1-success
        :exception RobotError : has error in robot which reported
        :exception CommRecvError : report this error after 5 time attempt to read data
        '''
        rtn = -1
        if COMMDEBUG:
            print('[INFO]TM_COM:This is debug mode, no status to read')
            return 1
        self.connect()
        for cnt in range(5):
            try:
                data = self.sockfd.recv(self.MAX_RECVDATA_SIZE)
                rtn = self.stateRT.parse_tcp_state_data(data)
                break
            except RobotError as exc:
                raise exc
            except RobotDataError:
                time.sleep(0.1)
                print('[INFO]TM_COM:Retry receive data!')
        self.disconnect()
        if rtn ==-1:
            raise CommRecvError(-1, '[ERROR]TM_COM:Error when receive robot data')
        return rtn

    def sendCommandMsg(self, cmd_msg, blocking = True):
        '''
        sendCommandMsg send a command message and also to read back robot state if blocking option is True
        :param cmd_msg: a string of message
        :param blocking: wait until command is finished by reading robot state
        :return: 1-success
        :exception RobotError: has error in robot which reported
        :NOTE: Please don't use this interface to check Gripper state
        '''
        ret = -1
        cmd_len=len(cmd_msg)+4
        if(cmd_len > self.MAX_SENDDATA_SIZE):
            print('[ERROR]TM_COM:Send data exceed max length')
            return ret
        header=struct.pack('>HBB',cmd_len,0,0)
        cmd=bytearray(header + str.encode(cmd_msg))
        if COMMDEBUG:
            print('[DEBUG]TM_COM: Command length:%d' %(cmd_len))
            print(cmd)
        else:
            self.connect()
            self.sockfd.send(cmd)
            self.disconnect()
        if blocking:
            time.sleep(0.2)
            try:
                while(not self.checkCommandFinished()):
                    time.sleep(0.3)
            except RobotError as exc:
                print('[ERROR]TM_COM:Robot error!')
                raise exc
        ret = 1
        return ret
    
    def checkCommandFinished(self):
        '''
        checkCommandFinished does the checking on robot state by try to read it
        :return:
            True: command is finished
            False: command is still in execution, or can't read robot state
        :exception RobotError: has error in robot which reported
        :NOTE: Please don't use this interface to check Gripper state
        '''
        try:
            rtn = self.readRobotState()
            return self.stateRT.rb_CommandDone
        except RobotError as exc:
            raise exc
        except CommRecvError:
            return False

class CommRecvError(Exception):
    def __init__(self, expression, message):
        self.expression = expression
        self.message = message

