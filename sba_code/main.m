clear
load data
%The meaning of the variables in the data
%c_left, c_right: principle point of left and right cameras.
%f_left, f_right: focal length of left and right cameras.
%RT: 
%kc_left, kc_right: The distortion parameters of left and right cameras.
%RT:    The exrtinsic parameter between the two camera.
%I_left I_right S0 P ref_global: same meaning in the function bundle_adjustment,
%                   please input 'help bundle_adjustment' for the details.
%left_image, right_image: The original 2d points in the left and right image.
%point_cloud: The 3d points constructed from the left_image and right_image
%               using stereo_triangualtion

   
%Note: the xml_io_tools and calibration toolbox are needed
%if you want to load calibration data in a xml file as I did. If you did
%   it manually, then you do not need to download the toolbox. See usage 2.
%
% Download xml_io_tools:
%       https://ww2.mathworks.cn/matlabcentral/fileexchange/12907-xml_io_tools
% Download calibration toolbox
%       http://www.vision.caltech.edu/bouguetj/calib_doc/download/index.html

%usage1: 
[S0_new, P_new, avg_distance] = bundle_adjustment(...
    S0,P,I_left,I_right,ref_global);
%usage2:
 [S0_new, P_new,avg_distance, F_new, C_new, KC_new] = bundle_adjustment(...
 S0,P, I_left, I_right, ref_global, 1,'both',1,0,...
 [f_left;f_right],[c_left;c_right],[kc_left;kc_right],RT);