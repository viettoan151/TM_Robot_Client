classdef RobotArm < handle
    %RobotArm Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        RBComm = RobotArmComm(true); %communication interface
        RBHomeJoints = single([105 20 100 -30 90 14]);
        RBBinJoints = single([60 40 90 -40 90 -30]);
        RBCameraPos = single([-5 400 500 180 -1 180]);
        RBToolOriBase = single([180.0, 0.0, 180.0]);
    end
    
    methods
        function obj = RobotArm(debug_m)
            %RobotArm Construct an instance of this class
            %   Detailed explanation goes here
            obj.RBComm = RobotArmComm(debug_m);
        end
        
        function rtn = RbA_Move_Joints(obj,joints)
            cmd_h = 'movj 0';
            cmd_b = join(string(joints),' ');
            cmd_t = '0';
            command = join([cmd_h cmd_b cmd_t],' ');
            rtn = obj.RBComm.sendCommand(char(command));
        end
        function rtn = RbA_Move_To(obj, tool_pos, tool_ori)
            cmd_h = 'cmd41 0';
            cmd_b1 = join(string(tool_pos),' ');
            
            %we have only top-down grasp, only correct grasp angle
            tool_ori(1:2) = obj.RBToolOriBase(1:2);
            %tool_ori(3) = obj.RBToolOriBase(3);
            cmd_b2 = join(string(tool_ori),' ');
            
            command = join([cmd_h cmd_b1 cmd_b2],' ');
            rtn = obj.RBComm.sendCommand(char(command));
        end
        function rtn = RbA_Move_To_XYAlpha(obj, posXY, angle)
            assert(isequal(size(posXY),[1 2]));
            obj.RBComm.readRobotState();
            curr_pos = obj.RBComm.RBST_ActToolPos;
            curr_pos(1:2)=posXY;
            %there is angle specific
            if (nargin >2)
                curr_pos(6)=angle;
            end
            rtn=obj.RbA_Move_To(curr_pos(1:3),curr_pos(4:6));
        end
        function rtn = RbA_Move_To_ZAlpha(obj, posZ, angle)
            obj.RBComm.readRobotState();
            curr_pos = obj.RBComm.RBST_ActToolPos;
            curr_pos(3)=posZ;
            %there is angle specific
            if (nargin >2)
                curr_pos(6)=angle;
            end
            rtn=obj.RbA_Move_To(curr_pos(1:3),curr_pos(4:6));
        end
        function rtn = RbA_Close_Gripper(obj)
            command = 'digop 8 1 0';
            obj.RBComm.sendCommand(char(command));
            %special pause for Gripper because there is no check for tool
            pause(0.7);
            obj.RBComm.readRobotState();
            if(obj.RBComm.RBST_ToolDigitalInput ~= 2)
                %grasp success
                rtn = true;
            else
                %No object
                rtn = false;
            end
        end
        function rtn = RbA_Open_Gripper(obj)
            command = 'digop 8 0 0';
            rtn = obj.RBComm.sendCommand(char(command));
            %special pause for Gripper because there is no check for tool
            pause(0.5);
        end
        function rtn = RbA_Clear_Error(obj)
            obj.RBComm.sendCommand('cmdc0');
            obj.RbA_Open_Gripper();
            obj.RbA_Go_Home();
            rtn = obj.RbA_Go_Camera();
        end
        function rtn = RbA_Go_Home(obj)
            rtn = obj.RbA_Move_Joints(obj.RBHomeJoints);
        end
        function rtn = RbA_Go_Bin(obj)
            rtn = obj.RbA_Move_Joints(obj.RBBinJoints);
        end
        function rtn = RbA_Go_Camera(obj)
            cmd_h = 'cmd41 0';
            cmd_b1 = join(string(obj.RBCameraPos),' ');
            command = join([cmd_h cmd_b1], ' ');
            rtn = obj.RBComm.sendCommand(char(command));
        end
    end
end

