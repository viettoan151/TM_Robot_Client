function IP_RGB_Image = IP_Get_Image(read_src)
%IP_Get_Image Use in Image Processing Assignment to get color RGB Image
%   Function will return image from Source: your local folder, or from Robot ARM
%   Source: - Default is ./resize folder
%           - 'Local' is same as Default
%           - 'RobotArm' is from Robot Arm
    source_str = './resize/';
    if (nargin < 1)
        read_src = 'RobotArm';
    else
        if strcmpi(read_src,'Local')
            read_src = source_str;
        elseif strcmpi(read_src,'RobotArm')
            read_src = 'RobotArm';
        else
            error('Source is not support');
        end
    end
    if strcmpi(read_src,'RobotArm')
        client = UEyeClient('192.168.0.11');
        img=client.UEyeClient_get_data();
        if(client.im_height>300)
            x=[170,1086]; %916
            y=[17,711]; %694
            img_cut=img(y(1):y(2),x(1):x(2),:);
            %image(img);
            img_rsz=imresize(img_cut,[300 400]);
            IP_RGB_Image = img_rsz;
        elseif(client.im_height==300)
            IP_RGB_Image=img;
        end
    else
        %Read from folder has not been implemented yet!
        %Please use your read image function: imread()
        error('Read from folder has not been implemented yet!');
    end
        
    
end

