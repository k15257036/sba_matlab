function [f_left,c_left,kc_left,f_right,c_right,kc_right,P] = read_calibration_xml(filename)
%[f_left,c_left,kc_left,f_right,c_right,kc_right,P] = read_calibration_xml(filename)
%Output:    f:  focal length
%           c:  principle point
%           kc: distortion parameter

xDoc = xml_read(filename);

in = xDoc.camera_left.intrinsic_parameters;
K_left = [in.fx,    0,    in.cx;
            0,    in.fy,  in.cy;
            0,       0,     1]  ;
f_left = [K_left(1,1);K_left(2,2)];
c_left = [K_left(1,3);K_left(2,3)];
dist = xDoc.camera_left.distcoffes;
kc_left = [dist.k1; dist.k2; dist.k3; dist.k4; dist.k5];

in = xDoc.camera_right.intrinsic_parameters;
K_right = [in.fx,    0,   in.cx;
            0,    in.fy,  in.cy;
            0,       0,     1 ];
f_right = [K_right(1,1);K_right(2,2)];
c_right = [K_right(1,3);K_right(2,3)];
        
dist = xDoc.camera_right.distcoffes;
kc_right = [dist.k1  dist.k2  dist.k3  dist.k4 dist.k5]';

R = xDoc.R;
R = [R.R1,  R.R2,   R.R3;
     R.R4,  R.R5,   R.R6;
     R.R7,  R.R8,   R.R9];

T = xDoc.T;
T = [T.T1;
     T.T2;
     T.T3];

P = [R,T];