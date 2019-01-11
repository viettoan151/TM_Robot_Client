# -*- coding: utf-8 -*-
from .tm_robot_state_rt import TmRobotStateRT

import socket
import threading
import selectors
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
    data_lock = threading.Condition()
    def __init__(self, data_condv_rt=None, ip='192.168.0.10', port=6188, timeout_ms=5000, timeval_ms=500):
        self.sockfd=None
        self.optflag=1
        self.thread_alive=False
        print("[DEBUG]TM_COM: TMCommunication Constructor")
        if COMMDEBUG:
            print("This is DEBUG MODE of communication module!!!!")
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
    
    def start(self):
        self.halt()
        #connect to server
        self.sockfd= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        '''Set socket options IPPROTO_TCP: TCP_NODELAY, TCP_QUICKACK
        SOL_SOCKET: SO_REUSEADDR
        self.sockfd.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY,1)
        #can't set TCP_QUICKACK in python
        self.sockfd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        '''
        self.sockfd.settimeout(self.connect_timeout_s)
        try:
            self.sockfd.connect((self.server_ip,self.server_port))
        except socket.error as exc:
            print('ERROR: Create connection: %s' % exc)
            return -1
        print("TM_COM: TM robot is Connected")
        #create selector for socket I/O ready
        self.sel = selectors.DefaultSelector()
        self.sel.register(self.sockfd, selectors.EVENT_READ)
        #create recieve thread
        action_thread = threading.Thread(target=self.threadFunction)
        action_thread.daemon = True
        action_thread.start()
        return 1
    
    def halt(self):
        self.thread_alive=False
        print("[INFO] TM_COM: halt\n")
        
    def disConnect(self):
        self.sockfd.close()
        print("TM_COM: Connection to TM robot CLOSED")
    
    def writeToBuf(self, data):
        #print("Write to buffer!")
        self.data_lock.acquire()
        self.data_ready = False
        temp = bytearray(data)
        #check data length
        self.stateRT.parse_tcp_state_data(temp, 'None')
        if(self.stateRT.data_length == self.CORRECT_RECVDATA_SIZE):
            self.data_bytes = temp
            self.data_ready = True
        self.data_lock.notify()
        self.data_lock.release()
        
    def readFromBuf(self):
        self.data_lock.acquire()
        while(not self.data_ready):
            self.data_lock.wait()
        ret = self.data_bytes.copy()
        self.data_lock.release()
        return ret
    
    def threadFunction(self):
        self.sb_H = 0
        self.sb_T = 0
        self.thread_alive=True
        while(self.thread_alive):
            events=self.sel.select(timeout = self.thread_timeval_s)
            if(len(events) > 0):
                for key,mask in events:
                    if mask&selectors.EVENT_READ:
                        data = self.sockfd.recv(self.MAX_RECVDATA_SIZE)
                        self.writeToBuf(data)
            else:
                if not self.thread_alive:
                    break
        self.disConnect();
        print("[INFO] TM_COM: Recv. thread finished\n");
    def sendCommandData(self, cmd_name, cmd_data):
        return 0
    
    def sendCommandMsg(self, cmd_msg, blocking = True):
        ret = -1;
        cmd_len=len(cmd_msg)+4
        if(cmd_len > self.MAX_SENDDATA_SIZE):
            return ret
        
        header=struct.pack('>HBB',cmd_len,0,0)
        cmd=bytearray(header + str.encode(cmd_msg))
        if(self.sockfd):
            print('Command length:%d' %(cmd_len))
            print(cmd)
            if not COMMDEBUG:
                if(COMMRUNMODE):
                    self.stateRT.command_done = False
                    self.sockfd.send(cmd)
                else:
                    c=input('[C]onfirm:')
                    if c=='c' or 'C':
                        self.sockfd.send(cmd)
                    else:
                        print('Cancelled command!')
            
            if blocking:
                time.sleep(0.2)
                while(not self.checkCommandFinished()):
                    time.sleep(0.3)
            ret = 1

        return ret
    
    def checkCommandFinished(self):
        cur_data = self.readFromBuf()
        self.stateRT.parse_tcp_state_data(cur_data,'robot_state')
        return self.stateRT.command_done

            

        
