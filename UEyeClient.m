classdef UEyeClient < handle
    properties
        im_height = 720;
        im_width = 1280;
        im_channel = 3;
        tcp_host_ip = '127.0.0.1';
        tcp_port = 60000;           %default free port for camera
        buffer_size = 4096;         %4k data
        camera_f = zeros(1,2);      %lense focus in pixel size
        camera_c = zeros(1,2);      %lense center point
	end
	methods
        function obj = UEyeClient(val)
            if (nargin < 1)
                obj.tcp_host_ip = "127.0.0.1";
            else
                if ischar(val)
                    obj.tcp_host_ip = val;
                else
                    error('Value must be string host');
                end
            end
        end
        function RGBImage = UEyeClient_get_data(obj)
            %create socket
            t_socket = tcpip(obj.tcp_host_ip, obj.tcp_port, 'NetworkRole', 'client',...
                 'InputBufferSize', obj.buffer_size, 'ByteOrder', 'littleEndian');
            %open socket
            fopen(t_socket);
            %send a ready signal to server 
            send_data='abcd';
            fwrite(t_socket, send_data);
            %{
            Receive back the data from server
            - Data from server in form:[Header  Image data]
            - Header:
                + uint32 + uint32 + uint32 for image size
                + float64*2 for camera lense focus 
                + float64*2 for camera center point
            - Image data is "BGRA of Python flattened"
            %}
            %Header part
            obj.im_height = fread(t_socket,1,'uint32');
            obj.im_width = fread(t_socket,1,'uint32');
            obj.im_channel = fread(t_socket,1,'uint32');
            obj.camera_f = fread(t_socket,[1,2],'float64');
            obj.camera_c = fread(t_socket,[1,2],'float64');
            %Image data part
            %From server, data is "BGRA of Python flattened",there for 
            %the channel will be 4. Repeatly get data from Server until end
            max_data_length=obj.im_height*obj.im_width*obj.im_channel;
            recv_data = zeros(max_data_length,1,'uint8');
            curr_data_length = 0;
            while(curr_data_length<max_data_length)
                next_recv_length = min((max_data_length-curr_data_length),obj.buffer_size);
                r= fread(t_socket,next_recv_length);
                %next_recv_length = min((max_data_length-curr_data_length),size(r,1));
                recv_data((curr_data_length+1):(curr_data_length+next_recv_length)) = ...
                        uint8(r(1:next_recv_length));
                curr_data_length = curr_data_length+next_recv_length;
            end
            fclose(t_socket); %close socket
            delete(t_socket);
            echotcpip('off');
            %Process the received data
            %Python [HxWxC] --> Matlab[CxWxH]
            if (obj.im_channel == 4) %BGRA
                img_BGRA_t = reshape(recv_data,[obj.im_channel,obj.im_width,obj.im_height]);
                %Color space BGRA[4xWxH] --> RGB[WxHx3]
                img_RGB_t = zeros(obj.im_width,obj.im_height,3,'uint8');
                img_RGB_t(:,:,1) = img_BGRA_t(3,:,:);
                img_RGB_t(:,:,2) = img_BGRA_t(2,:,:);
                img_RGB_t(:,:,3) = img_BGRA_t(1,:,:);
                obj.im_channel = 3;
                %Do transpose RGB[WxHx3] --> RGB[HxWx3]
                RGBImage = permute(img_RGB_t,[2 1 3]);
            elseif(obj.im_channel == 3) %RGB
                img_RGB_t = reshape(recv_data,[obj.im_channel,obj.im_width,obj.im_height]);
                RGBImage = permute(img_RGB_t,[3 2 1]);
            end
        end
    end
end