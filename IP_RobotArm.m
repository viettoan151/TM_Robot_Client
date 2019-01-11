classdef IP_RobotArm < handle
    %IP_ROBOTARM Robot Arm class used for Image Processing course
    %   Class provide two main interfaces for final project:
    %   - IP_RBA_Take_Picture: Robot goes to take workspace picture
    %   - IP_RBA_Grasp_Object_PP: Robot goes to grasp object
    %Student is supposed to use above interfaces to complete final project
    %
    %NOTICE: DON'T CHANGE THIS FILE, ROBOT WILL ACT UNEXPECTEDLY
    %        ALWAYS KEEP YOUR DISTANCE WITH ROBOT WHEN CALL FUNCTIONS
    
    properties
        DEBUG_MODE = true;
        WorkSpace=([[-200 360 -120];...
                    [ 200 660  100]]); %workspace limitation is in milimeter
        ObjHeight=50;%default object height is 5cm
        UpHeight=150;%default object height is 15cm
        GripperOffset = 120;%distance from endeffector to last joint
        AngleOffset = 180;%true robot angle offset when work with above workspace
        Pixel2mMeter = 1;%1pixel = 1 mm
        ImageSpace=[[0     0];...    
                    [400 300]]; %Image from 
        Robot = RobotArm(true);
    end
    
    methods
        function obj = IP_RobotArm(debug_m)
            %IP_ROBOTARM Construct an instance of this class
            %   Detailed explanation goes here
            obj.Robot = RobotArm(debug_m);
            obj.DEBUG_MODE = debug_m;
        end
        function grasp_result = IP_RBA_Grasp_Object_PP(obj, p1, p2)
            %IP_RBA_Grasp_Object_PP: Grasp an object in image coordinate
            %p1,p2: the pixel coordinate in Image [x y]
            
            %two points must be in format of vector [x y]
            assert(isequal(size(p1),[1 2]));
            assert(isequal(size(p2),[1 2]));
            
            %distance between two points 
            distance = sqrt(sum((p1-p2).^2));
            %Two points must far aways enough
            if(distance < 20)
                disp('Two points have too small distance!');
                grasp_result = false;
                return;
            end
            
            %Get center point/grasp point
            center = round((p1+p2)/2);
            %Limit center point in image space
            if((center(1)<obj.ImageSpace(1,1))||(center(1)>obj.ImageSpace(2,1))||...
                    (center(2)<obj.ImageSpace(1,2)) || (center(2)> obj.ImageSpace(2,2)))
                disp('Grasp point is outside workspace!');
                grasp_result = false;
                return;
            end
            %Calculte angle created with Ox axist from two point
            if(abs(p1(1)-p2(1))<2)  %tolerance is 2 pixel
                angle_with_Ox = 90; %Perpendicular
            elseif(abs(p1(2)-p2(2))<2)
                angle_with_Ox = 0;  %Parallel
            else
                %swap two points to get the ray, angle in range (-pi/2,
                %pi/2)
                if(p1(1)>p2(1))
                    temp = p1;
                    p1=p2;
                    p2=temp;
                end
                dy = p1(2)-p2(2);
                angle_with_Ox = -rad2deg(asin(dy/distance));
            end
            %convert from image base to world base
            angle_world = obj.AngleOffset - angle_with_Ox;
            center_world = center * obj.Pixel2mMeter; %scale
            center_world(1) = center_world(1) + obj.WorkSpace(1,1);
            center_world(2) = - center_world(2) + obj.WorkSpace(2,2);
            fprintf('[INFO]Grasp at:(%.1f,%.1f)mm, with angle:{%.2f}Deg!\n',...
                    center_world,angle_world);
            grasp_result = obj.IP_RBA_Grasp_Object_2DAlpha(center_world,angle_world);
        end
        function rtn_image = IP_RBA_Take_Picture(obj)
            %IP_RBA_Take_Picture take whole workspace picture
            %rtn_image: RGB image h:300 w:400
            try
                %Let Robot go to above object
                obj.Robot.RbA_Go_Home();
                obj.Robot.RbA_Go_Camera();
                rtn_image = IP_Get_Image();
                obj.Robot.RbA_Go_Home();
            catch ME
                disp(ME.message);
                obj.Robot.RbA_Clear_Error();
                error('[ERROR]:Robot take picture has error!');
            end
        end
        function rtn = IP_RBA_Drop_Object_Bin(obj)
            %IP_RBA_Drop_Object: Drop object after grasp it
            try
                grasp_st = obj.Robot.RbA_Close_Gripper();
                if(grasp_st)
                    obj.Robot.RbA_Go_Home();
                    obj.Robot.RbA_Go_Bin();
                    obj.Robot.RbA_Open_Gripper();
                    obj.Robot.RbA_Go_Home();
                    rtn=1;
                else
                    obj.Robot.RbA_Open_Gripper();
                    obj.Robot.RbA_Go_Home();
                    rtn=0;
                end
            catch ME
                disp(ME.message);
                obj.Robot.RbA_Clear_Error();
                error('[ERROR]:Robot drop object has error!');
            end
        end
        function rtn = IP_RBA_Drop_Object_Origin(obj)
            %IP_RBA_Drop_Object: Drop object after grasp it
            try
                grasp_st = obj.Robot.RbA_Close_Gripper();
                if(grasp_st)
                    obj.Robot.RbA_Move_To_ZAlpha(obj.WorkSpace(1,3) + ...
                        obj.ObjHeight + obj.GripperOffset + 20);
                    obj.Robot.RbA_Open_Gripper();
                    obj.Robot.RbA_Move_To_ZAlpha(obj.UpHeight + ...
                        obj.GripperOffset + obj.WorkSpace(1,3) - 20, angle);
                    obj.Robot.RbA_Go_Home();
                    rtn=1;
                else
                    obj.Robot.RbA_Open_Gripper();
                    obj.Robot.RbA_Go_Home();
                    rtn=0;
                end
            catch ME
                disp(ME.message);
                obj.Robot.RbA_Clear_Error();
                error('[ERROR]:Robot drop object has error!');
            end
        end
        function rtn = IP_RBA_Open_Gripper(obj)
            rtn = obj.Robot.RbA_Open_Gripper();
        end
        function rtn = IP_RBA_Close_Gripper(obj)
            rtn = obj.Robot.RbA_Close_Gripper();
        end
        function rtn = IP_RBA_Go_Home(obj)
            rtn = obj.Robot.RbA_Go_Home();
        end
    end
    
    methods (Access = private)
        function grasp_result = IP_RBA_Grasp_Object_2DAlpha(obj,position, angle)
            %IP_RBA_Grasp_Object_2DAlpha: Grasp an object in world coordinate
            %position: the 2D coordinate in workspace limted (x,y)
            %angle: 'world coordinate angle' to grasp object(Degree)
            
            %constrain grasp position inside workspace
            position(1) = min(max(position(1),obj.WorkSpace(1,1)),obj.WorkSpace(2,1));
            position(2) = min(max(position(2),obj.WorkSpace(1,2)),obj.WorkSpace(2,2));
            %convert grasp angle in range [90 270]
            angle = mod(angle,360);
            if(angle < 90)
                angle = angle + 180;
            elseif(angle > 270)
                angle = angle - 180;
            end
            try
                %Let Robot go to above object
                obj.Robot.RbA_Go_Home();
                obj.Robot.RbA_Move_To_XYAlpha(position, angle);
                %Grasp 
                obj.Robot.RbA_Open_Gripper();
                obj.Robot.RbA_Move_To_ZAlpha(obj.ObjHeight+obj.GripperOffset...
                        +obj.WorkSpace(1,3)-20, angle);
                obj.Robot.RbA_Close_Gripper();
                %Lift up
                obj.Robot.RbA_Move_To_ZAlpha(obj.UpHeight+obj.GripperOffset ...
                        +obj.WorkSpace(1,3)-20, angle);
                grasp_result = obj.Robot.RbA_Close_Gripper();
            catch ME
                disp(ME.message);
                obj.Robot.RbA_Clear_Error();
                disp('[ERROR]:Robot grasp has error!');
                grasp_result = false;
            end
        end
    end
end

