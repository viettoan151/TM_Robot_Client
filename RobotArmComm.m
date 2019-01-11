classdef RobotArmComm < handle
    %ROBOTARMCOMM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        DEBUG_MODE = true;
        CORRECT_RECVDATA_SIZE = 372;
        MAX_SENDDATA_SIZE = 512;
        MAX_RECVDATA_SIZE = 1024;
        BUFF_EMPTY_FLAG = '000000FC';
        ServerIpAddr='192.168.0.10';
        ServerPort=6188;
        RBST_ActJointsPos = zeros(1,6,'single');
        RBST_CmdJointsPos = zeros(1,6,'single');
        RBST_ActToolPos = zeros(1,6,'single');
        RBST_ToolDigitalInput = uint16(0);
        RBST_BuffEmptyFlag = uint32(0);
        RBST_ErrorCode_0 = uint8(0);
        RBST_ErrorCode_1 = uint8(0);
    end
    
    methods
        function obj = RobotArmComm(debug_m)
            %ROBOTARM Construct an instance of this class
            %   Detailed explanation goes here
            if (nargin < 1)
                obj.DEBUG_MODE = true;
            else
                obj.DEBUG_MODE = debug_m;
            end
            obj.RBST_BuffEmptyFlag = uint32(hex2dec(obj.BUFF_EMPTY_FLAG));
        end
        function rtn = sendCommand(obj,command)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if ~ischar(command)
                error('ERROR:Command is not in char array!');
            end
            
            cmd_len = uint16(size(command,2) + 4);
            header = typecast(swapbytes(uint16(cmd_len)), 'uint8');
            send_data= uint8([header 0 0 command]);
            if obj.DEBUG_MODE
                disp(dec2hex(send_data));
                rtn = 1;
            else
                t_socket = tcpip(obj.ServerIpAddr, obj.ServerPort, 'NetworkRole', 'client',...
                     'InputBufferSize', obj.MAX_RECVDATA_SIZE, 'ByteOrder', 'littleEndian');
                %open socket
                fopen(t_socket);
                %send a ready signal to server 
                fwrite(t_socket, send_data);
                fclose(t_socket); %close socket
                delete(t_socket);
                echotcpip('off');
                
                %check robot arm is done above command
                pause(0.1);
                while(1)
                    try
                        rtn = obj.readRobotState();
                        if rtn == -1
                            %read failed
                            disp('[WARN]Can''t read from Robot state');
                            break;
                        elseif rtn == 0
                            %robot is still busy
                            pause(0.5);
                        elseif rtn == 1
                            %done
                            break;
                        else
                            error('[ERROR]Undefined return value of Robot state');
                        end
                    catch ME
                        rethrow(ME)
                    end
                end
            end
        end
        
        function rtn = readRobotState(obj)
            if obj.DEBUG_MODE
                disp('[INFO]This is debug mode, no read status!');
                rtn = 1;
            else
                t_socket = tcpip(obj.ServerIpAddr, obj.ServerPort, 'NetworkRole', 'client',...
                     'InputBufferSize', obj.MAX_RECVDATA_SIZE, 'ByteOrder', 'littleEndian');
                %open socket
                fopen(t_socket);
                %send a ready signal to server 
                cnt=1;
                recv_flag=false;
                data_len=0;
                robot_data=zeros(obj.CORRECT_RECVDATA_SIZE,1);
                while(cnt < 5)
                    recv_data = fread(t_socket);
                    if(size(recv_data,1) > 1)
                        data_len = typecast(uint8([recv_data(2) recv_data(1)]),'uint16');
                        if(data_len == obj.CORRECT_RECVDATA_SIZE)
                            robot_data = recv_data(1:obj.CORRECT_RECVDATA_SIZE);
                            recv_flag = true;
                            break;
                        end
                    end
                    cnt  = cnt + 1;
                end
                fclose(t_socket); %close socket
                delete(t_socket);
                echotcpip('off');
                if recv_flag
                    try
                        rtn = obj.parseRobotData(robot_data);
                    catch ME
                        rethrow(ME);
                    end
                else
                    rtn = -1;
                end
            end
        end
        function rtn = parseRobotData(obj, robot_data)
            MLIDX = 1;
            obj.RBST_ErrorCode_0 = uint8(robot_data(370+MLIDX));
            obj.RBST_ErrorCode_1 = uint8(robot_data(371+MLIDX));
            %actual joints data
            byte_idx = 12 + MLIDX;
            for i=1:6
                obj.RBST_ActJointsPos(i)=typecast(uint8(robot_data(byte_idx:(byte_idx+3))),'single');
                byte_idx = byte_idx + 4;
            end
            %command joints data
            byte_idx = 36 + MLIDX;
            for i=1:6
                obj.RBST_CmdJointsPos(i)=typecast(uint8(robot_data(byte_idx:(byte_idx+3))),'single');
                byte_idx = byte_idx + 4;
            end
            %actual tool position
            byte_idx = 204 + MLIDX;
            for i=1:6
                obj.RBST_ActToolPos(i)=typecast(uint8(robot_data(byte_idx:(byte_idx+3))),'single');
                byte_idx = byte_idx + 4;
            end
            %Tool digital input
            byte_idx = 356 + MLIDX;
            obj.RBST_ToolDigitalInput=uint16(typecast(uint8(robot_data(byte_idx)),'uint8'));
            %Command buffer status
            byte_idx = 366 + MLIDX;
            obj.RBST_BuffEmptyFlag=typecast(uint8(robot_data(byte_idx:(byte_idx+3))),'uint32');
            %error code
            if(obj.RBST_ErrorCode_0~=0)
                %error in robot
                fprintf('[ERROR]Robot error: 0x%X 0x%X.\n',obj.RBST_ErrorCode_0, obj.RBST_ErrorCode_1);
                ME = MException('ROBOT: Error', ...
                        'Robot error code:%X %X',obj.RBST_ErrorCode_0, obj.RBST_ErrorCode_1);
                throw(ME)
            elseif(obj.RBST_BuffEmptyFlag ~= uint32(hex2dec(obj.BUFF_EMPTY_FLAG)))
                %robot is still running
                rtn = 0;
            else
                %robot is done
                rtn = 1;
            end
        end
    end
end

