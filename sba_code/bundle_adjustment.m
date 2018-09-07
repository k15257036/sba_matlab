function [S0_new, P_new, avg_distance, F_new, C_new, KC_new] = bundle_adjustment(...
    S0,P,I_left,I_right,ref_global,miu,type,change_intrinsic,filename,...
    F,C,KC,RT)
%           [S0_new, P_new,avg_distance, F_new, C_new, KC_new] 
%           = bundle_adjustment(
%           S0, ,P, I_left, I_right, ref_global, 
%           miu,type, change_intrinsic,  filename,F,C,KC)
%
%           Using bundle adjustment to minimize the reprojection error of
%           image points. M image, N feature points.
%
%Input:     S0: The estimated 3d points.
%               Type: matrix, size: 3*N
%           P:  The transformation matrix from global 3d points to the
%               local.
%               Type: cell, size: 3*4(matrix,[R T]) of 1*M(cell)
%           I_left: Local 2d points of left image, including visible and 
%                   invisible points. 
%               Type: cell, size: 2*N(matrix) of 1*M(cell)
%           I_right: Local 2d points of right image, including visible and 
%                   invisible points. If no need of I_right, make it to
%                   zero. (The left and right can not be zero at same time)
%               Type: cell, size: 2*N(matrix) of 1*M(cell)
%           ref_global: The information of the local points'(or image)
%               index in the global points(image).
%               Type: matrix, size: N*M
%           miu:    The parameter to adjust LM: (JTJ +miu*I) = JTeps
%                   Default: 1
%           Type:   There are three type:‘both’,'point','parameter'
%                   'both': points and parameters are both adjusted
%                   'point': only points will be adjusted
%                   'parameter': only other parameters except points will
%                                be adjusted.
%                   Default: 'both'
%           change_intrinsic:    Whether change camera intrinsic parameter
%               such as F, C, KC. nonzero means change, and zero means not.
%               Default: 0
%           filename:   the name of the camera calibration file. It has the
%               intrinsic and extrinsic parameters.
%               Default: 'CameraCalibration.xml'
%           F:  The focal length of the camera(s).
%               Type:   matrix
%               Size:   ①2*1 It means left and right camera have the same
%                       focal length for every image.
%                       ②4*1 The first two rows are the focal length of the
%                       left camera, and the last two rows are the right.
%                       Every image will share the same focal length.
%                       ③2*M Left and the right camera have the same focal
%                       length, which is different in differnt images.
%                       ④4*M Left and the right camera have different focal
%                       length, and it is different in differnt images.
%           C:  The principle point's coordinates of the camera(s)
%               Type:   matrix
%               Size:   ①2*1 ②4*1 ③2*M ④4*M
%           KC: The distortion parameters of the camera(s).
%               Type:   matrix
%               Size:   ①5*1 ②10*1 ③5*M ④10*M
%           RT: The exrtinsic parameter between the two camera.
%               Type: matrix, size:3*4
%       Note: If you want to input camera intrinsic parameter
%       mannually(rather than using camera calibration file, no parameter
%       of F, C, KC or RT can be omitted. Their size should have same type.
%
%           
%Output:    S0_new: The updated value of S0, same type and size as S0.
%           P_new:  The updated value of P, same type and size as P.
%           F_new:  The updated value of focal length.
%                   Type: matrix, size: 4*M. The first 2 rows are about
%                   left camera, the following two 2 are about the
%                   right.
%           C_new:  The updated value of principle length.
%                   Type: matrix, size: 4*M. The first 2 rows are about
%                   left camera, the following 2 rows are about the
%                   right.
%           KC_new: The updated value of distortion parameter.
%                   Type: matrix, size: 10*M. The first 5 rows are about
%                   left camera, the following 5 rows are about the
%                   right.
%
%       Note:      If change_intrinsic's value is 0, then F_new, C_new, KC_new
%           will return the value of the calibration result with the size
%           of 4*1([fx_left;fy_left;fx_right;fy_right])
%           4*1([cx_left;cy_left;cx_right;cy_right])
%           10*1([kc1_left;..;kc5_left];kc1_right;..;kc5_right])
%           seperately.
%           If change_intrinsic's value is 1, then F_new, C_new, KC_new
%           will return the updated value of the calibration result with
%           the size of 4*M, 4*M, 10*M seperately. The parameter M means
%           different image.
%
%Eg:    ①No willing to change intrinsic parameter, and the calibration result
%         are in the specific xml file:
%         [S0_new, P_new, avg_distance] = bundle_adjustment(...
%         S0,P,I_left,I_right,ref_global)
%       ②Willing to change the intrinsic parameter, and input calibration
%         result mannually:
%         [S0_new, P_new,avg_distance, F_new, C_new, KC_new] 
%          = bundle_adjustment(...
%         S0,P, I_left, I_right, ref_global,... 
%           miu,type,1,0,F,C,KC)


if nargin < 9
    filename = 'CameraCalibration.xml';
    if nargin < 8
        change_intrinsic = 0;
        if nargin < 7
            type = 'both';
            if nargin < 6
                miu = 1;
            end
        end
    end
end


%Judge the information input is valid.
if size(I_left,2)==0||size(I_right,2)==0
    disp('The image input is empty, please check it');
    return
end

if size(I_left,2)~=size(I_right)
    disp('The number of left image does not equal to the right');
    return;
end

N = size(S0,2); %N is the number of the feature point
M = max(size(I_left,2),size(I_right,2));  %M is the number of the image gathered.

for j = 1:M
    if size(I_left{j},2)~=size(I_right{j},2)
        disp('The point''s number of image: ');
        disp(j)
        disp('is not equal between the left and right');
        return;
    end
end

if nargin > 9
    if size(F,2)~=size(C,2)||size(C,2)~=size(KC,2)
        disp('The size of F C kC is not in same type');
        return
    end
    if size(F,1)~=2&&size(F,1)~=4
        disp('The number of rows of F must be 2 or 4');
        return
    end
    if size(C,1)~=2&&size(C,1)~=4
        disp('The number of rows of C must be 2 or 4');
        return
    end
    if size(KC,1)~=5&&size(KC,1)~=10
        disp('The number of rows of KC must be 5 or 10');
        return
    end
end

change_intrinsic = logical(change_intrinsic);
        
%Load calibration result
%F_left:  Focal length of the left camera.
%   Type: matrix, size: 2*1([fx;fy]) or 2*M
%   If its size is 2*M, it means the Focal length of each image
%   does not need to be adjusted.
%C_left:  Principle point of the left camera.
%   Type: matrix, size: 2*1([cx;cy]) or 2*M
%   If its size is 2*M, it means the priciple point of each
%   image does not need to be adjusted.
%KC_left:    Distortion parameters of the left camera.
%   Type: matrix, size: 5*1([kc1;kc2;kc3;kc4;kc5]) or 5*M
%   If its size is 5*M, it means the distortion parameters of 
%   each image does not need to be adjusted.
%F_right:    Focal length of the right camera.
%C_right:    Principle point of the right camera.          
%KC_right:   Distortion parameters of the right camera.
%RT:    The transformation matrix from left camera reference
%       coordinates to the right camera.
%       Type: matrix, size: 3*4([R T])

if nargin < 9
    [F_left,C_left,KC_left,F_right,C_right,KC_right,RT] = read_calibration_xml(filename);
    F_left = repmat(F_left,1,M);
    C_left = repmat(C_left,1,M);
    KC_left = repmat(KC_left,1,M);
    F_right = repmat(F_right,1,M);
    C_right = repmat(C_right,1,M);
    KC_right = repmat(KC_right,1,M);
else
    if size(F,1) == 2
        [F_left,F_right] = deal(F);
        [C_left,C_right] = deal(C);
        [KC_left,KC_right] = deal(KC);
    else
        F_left = F(1:2,:);
        F_right = F(3:4,:);
        C_left = C(1:2,:);
        C_right = C(3:4,:);
        KC_left = KC(1:5,:);
        KC_right = KC(6:10,:);
    end
    if size(F,2) == 1
        F_left = repmat(F_left,1,M);
        F_right = repmat(F_right,1,M);
        C_left = repmat(C_left,1,M);
        C_right = repmat(C_right,1,M);
        KC_left = repmat(KC_left,1,M);
        KC_right = repmat(KC_right,1,M);
    end
end
    




%delta is used to judege whether some global point is visible 
%   in the local image.
delta = logical(ref_global);

%P_om is the rodrigues format of rotation matrix from S0 to S
%P_T is the transition vector from S0 to S
[P_om,P_T] = deal(zeros(3,M));
for j = 1:M
    P_om(:,j) = rodrigues(P{j}(:,1:3));
    P_T(:,j) = P{j}(:,4);
end

%For left image,from S0 to xij: P_R*S0+P_T -> xij
%       P_om = rodrigues(P_R)
%       <-> project_points2(S0,P_om,P_T)
%       dx/dom = dx/dP_om
%       dx/dT = dx/dP_T
%
%For right image, from S0 to xij:  R*(P_R*S0+P_T)+T -> xij
%       P_om_right = rodrigues(R*P_R)
%       P_T_right = R*P_T+T
%       <-> project_points2(S0,om_right,T_right)
%       condition: om1 + om2 is approximated to rodrigues(R1*R2)
%           dx/dom_right = dx/d(P_om+R_om) = dx/dom
%           dx/dT_right = dx/d(R*P_T+T) = dx/d(R*P_T) = dx/dP_T * 1/R
%           ->  dx/dT = dx/dP_T = dx/dT_right * R
[P_om_right,P_T_right] = deal(zeros(3,M));
for j = 1:M
    P_om_right(:,j) = rodrigues(RT(:,1:3)*P{j}(:,1:3));
    P_T_right(:,j) = RT(:,1:3)*P{j}(:,4)+RT(:,4);
end


%x_p is the reprojection 2d points of each image.
x_p = cell(2,M);
for j = 1:M
    x_p{1,j} = project_points2(S0,P_om(:,j),P_T(:,j),F_left(:,j),C_left(:,j),KC_left(:,j));
    x_p{1,j} = x_p{1,j}.*delta(:,j)';
    x_p{2,j} = project_points2(S0,P_om_right(:,j),P_T_right(:,j),F_right(:,j),C_right(:,j),KC_right(:,j));
    x_p{2,j} = x_p{2,j}.*delta(:,j)';
end
% [~,dxdom,dxdT,dxdf_left,dxdc_left,dxdk_left] = project_points2(S0,P_om(:,j),P_T(:,j),F_left(:,j),C_left(:,j),KC_left(:,j));
% [~,dxdom_right,dxdT_right,dxdf_right,dxdc_right,dxdk_right] = project_points2(S0,P_om_right(:,j),P_T_right(:,j),F_right(:,j),C_right(:,j),KC_right(:,j));


A = cell(N,M);
%A{:,1:M}  =       dx_left/dom  dx_left/dT  dx_left/df_left     dx_left/dc_left dx_right/dk_left 
%                       3           3            2                    2             5       
%                      1:3         4:6          7:8                  9:10          11:15      

for j = 1:M
    [~,dxdom,dxdT,dxdf_left,dxdc_left,dxdk_left] = project_points2(S0,P_om(:,j),P_T(:,j),F_left(:,j),C_left(:,j),KC_left(:,j));
    [~,dxdom_right,dxdT_right,dxdf_right,dxdc_right,dxdk_right] = project_points2(S0,P_om_right(:,j),P_T_right(:,j),F_right(:,j),C_right(:,j),KC_right(:,j));
    for i = 1:N
        A{i,j} = zeros(4,24);
        if delta(i,j) == 1 && (strcmp(type,'both') || strcmp(type,'parameter'))
            A{i,j}(1:2,1:3) = dxdom(2*i-1:2*i,:);
            A{i,j}(1:2,4:6) = dxdT(2*i-1:2*i,:);
            A{i,j}(1:2,7:8) = dxdf_left(2*i-1:2*i,:)*change_intrinsic;
            A{i,j}(1:2,9:10) = dxdc_left(2*i-1:2*i,:)*change_intrinsic;
            A{i,j}(1:2,11:15) = dxdk_left(2*i-1:2*i,:)*change_intrinsic;
            
            A{i,j}(3:4,1:3) = dxdom_right(2*i-1:2*i,:);
            A{i,j}(3:4,4:6) = dxdT_right(2*i-1:2*i,:)*RT(:,1:3);
            A{i,j}(3:4,16:17) = dxdf_right(2*i-1:2*i,:)*change_intrinsic;
            A{i,j}(3:4,18:19) = dxdc_right(2*i-1:2*i,:)*change_intrinsic;
            A{i,j}(3:4,20:24) = dxdk_right(2*i-1:2*i,:)*change_intrinsic;
            
            
        end
    end
end
%A{:,M+1:2*M}  =      dx_right/dom  dx_right/dT   dx_right/df_right     dx_right/dc_right   dx_right/dk_right
%                            3          3                 2                    2                    5       
%                           1:3        4:6              16:17                18:19                20:24

        
B = cell(N,M);
%B{:,1:M} = dx_left/dS0

for j = 1:M
    for i = 1:N
        B{i,j} = zeros(4,3);
        if delta(i,j) == 1 && (strcmp(type,'both') || strcmp(type,'point'))
            R = P{j}(:,1:3);
            r11 = R(1,1);r12 = R(1,2);r13 = R(1,3);
            r21 = R(2,1);r22 = R(2,2);r23 = R(2,3);
            r31 = R(3,1);r32 = R(3,2);r33 = R(3,3);
            T = P_T(:,j);
            t1 = T(1);t2 = T(2); t3 = T(3);
            x = S0(1,i);
            y = S0(2,i);
            z = S0(3,i);
        %for left image
            fx = F_left(1,j);
            fy = F_left(2,j);
            kc1 = KC_left(1,j);
            kc2 = KC_left(2,j);
            kc3 = KC_left(3,j);
            kc4 = KC_left(4,j);
            kc5 = KC_left(5,j);
            %The following sustitution is made for less computation
            X_x_new = t1 + r11*x + r12*y + r13*z;
            X_y_new = t2 + r21*x + r22*y + r23*z;
            X_z_new = t3 + r31*x + r32*y + r33*z;
            X_z_new2 = X_z_new^2;
            X_z_new3 = X_z_new^3;
            %The following result is got from jacobian. No need care for the
            %detail.
            B{i,j}(1,1) = fx*(kc4*((6*r11*X_x_new)/X_z_new2 + (2*r21*X_y_new)/X_z_new2 - (6*r31*X_x_new^2)/X_z_new3 - (2*r31*X_y_new^2)/X_z_new3) + ((kc1*((2*r11*X_x_new)/X_z_new2 + (2*r21*X_y_new)/X_z_new2 - (2*r31*X_x_new^2)/X_z_new3 - (2*r31*X_y_new^2)/X_z_new3) + 3*kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2*((2*r11*X_x_new)/X_z_new2 + (2*r21*X_y_new)/X_z_new2 - (2*r31*X_x_new^2)/X_z_new3 - (2*r31*X_y_new^2)/X_z_new3) + 2*kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)*((2*r11*X_x_new)/X_z_new2 + (2*r21*X_y_new)/X_z_new2 - (2*r31*X_x_new^2)/X_z_new3 - (2*r31*X_y_new^2)/X_z_new3))*X_x_new)/X_z_new + (r11*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new + (2*kc3*r21*X_x_new)/X_z_new2 + (2*kc3*r11*X_y_new)/X_z_new2 - (r31*X_x_new*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new2 - (4*kc3*r31*X_x_new*X_y_new)/X_z_new3);
            B{i,j}(1,2) = fx*(kc4*((6*r12*X_x_new)/X_z_new2 + (2*r22*X_y_new)/X_z_new2 - (6*r32*X_x_new^2)/X_z_new3 - (2*r32*X_y_new^2)/X_z_new3) + ((kc1*((2*r12*X_x_new)/X_z_new2 + (2*r22*X_y_new)/X_z_new2 - (2*r32*X_x_new^2)/X_z_new3 - (2*r32*X_y_new^2)/X_z_new3) + 3*kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2*((2*r12*X_x_new)/X_z_new2 + (2*r22*X_y_new)/X_z_new2 - (2*r32*X_x_new^2)/X_z_new3 - (2*r32*X_y_new^2)/X_z_new3) + 2*kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)*((2*r12*X_x_new)/X_z_new2 + (2*r22*X_y_new)/X_z_new2 - (2*r32*X_x_new^2)/X_z_new3 - (2*r32*X_y_new^2)/X_z_new3))*X_x_new)/X_z_new + (r12*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new + (2*kc3*r22*X_x_new)/X_z_new2 + (2*kc3*r12*X_y_new)/X_z_new2 - (r32*X_x_new*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new2 - (4*kc3*r32*X_x_new*X_y_new)/X_z_new3);
            B{i,j}(1,3) = fx*(kc4*((6*r13*X_x_new)/X_z_new2 + (2*r23*X_y_new)/X_z_new2 - (6*r33*X_x_new^2)/X_z_new3 - (2*r33*X_y_new^2)/X_z_new3) + ((kc1*((2*r13*X_x_new)/X_z_new2 + (2*r23*X_y_new)/X_z_new2 - (2*r33*X_x_new^2)/X_z_new3 - (2*r33*X_y_new^2)/X_z_new3) + 3*kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2*((2*r13*X_x_new)/X_z_new2 + (2*r23*X_y_new)/X_z_new2 - (2*r33*X_x_new^2)/X_z_new3 - (2*r33*X_y_new^2)/X_z_new3) + 2*kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)*((2*r13*X_x_new)/X_z_new2 + (2*r23*X_y_new)/X_z_new2 - (2*r33*X_x_new^2)/X_z_new3 - (2*r33*X_y_new^2)/X_z_new3))*X_x_new)/X_z_new + (r13*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new + (2*kc3*r23*X_x_new)/X_z_new2 + (2*kc3*r13*X_y_new)/X_z_new2 - (r33*X_x_new*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new2 - (4*kc3*r33*X_x_new*X_y_new)/X_z_new3);
            B{i,j}(2,1) = fy*(kc3*((2*r11*X_x_new)/X_z_new2 + (6*r21*X_y_new)/X_z_new2 - (2*r31*X_x_new^2)/X_z_new3 - (6*r31*X_y_new^2)/X_z_new3) + ((kc1*((2*r11*X_x_new)/X_z_new2 + (2*r21*X_y_new)/X_z_new2 - (2*r31*X_x_new^2)/X_z_new3 - (2*r31*X_y_new^2)/X_z_new3) + 3*kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2*((2*r11*X_x_new)/X_z_new2 + (2*r21*X_y_new)/X_z_new2 - (2*r31*X_x_new^2)/X_z_new3 - (2*r31*X_y_new^2)/X_z_new3) + 2*kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)*((2*r11*X_x_new)/X_z_new2 + (2*r21*X_y_new)/X_z_new2 - (2*r31*X_x_new^2)/X_z_new3 - (2*r31*X_y_new^2)/X_z_new3))*X_y_new)/X_z_new + (r21*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new + (2*kc4*r21*X_x_new)/X_z_new2 + (2*kc4*r11*X_y_new)/X_z_new2 - (r31*X_y_new*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new2 - (4*kc4*r31*X_x_new*X_y_new)/X_z_new3);
            B{i,j}(2,2) = fy*(kc3*((2*r12*X_x_new)/X_z_new2 + (6*r22*X_y_new)/X_z_new2 - (2*r32*X_x_new^2)/X_z_new3 - (6*r32*X_y_new^2)/X_z_new3) + ((kc1*((2*r12*X_x_new)/X_z_new2 + (2*r22*X_y_new)/X_z_new2 - (2*r32*X_x_new^2)/X_z_new3 - (2*r32*X_y_new^2)/X_z_new3) + 3*kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2*((2*r12*X_x_new)/X_z_new2 + (2*r22*X_y_new)/X_z_new2 - (2*r32*X_x_new^2)/X_z_new3 - (2*r32*X_y_new^2)/X_z_new3) + 2*kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)*((2*r12*X_x_new)/X_z_new2 + (2*r22*X_y_new)/X_z_new2 - (2*r32*X_x_new^2)/X_z_new3 - (2*r32*X_y_new^2)/X_z_new3))*X_y_new)/X_z_new + (r22*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new + (2*kc4*r22*X_x_new)/X_z_new2 + (2*kc4*r12*X_y_new)/X_z_new2 - (r32*X_y_new*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new2 - (4*kc4*r32*X_x_new*X_y_new)/X_z_new3);
            B{i,j}(2,3) = fy*(kc3*((2*r13*X_x_new)/X_z_new2 + (6*r23*X_y_new)/X_z_new2 - (2*r33*X_x_new^2)/X_z_new3 - (6*r33*X_y_new^2)/X_z_new3) + ((kc1*((2*r13*X_x_new)/X_z_new2 + (2*r23*X_y_new)/X_z_new2 - (2*r33*X_x_new^2)/X_z_new3 - (2*r33*X_y_new^2)/X_z_new3) + 3*kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2*((2*r13*X_x_new)/X_z_new2 + (2*r23*X_y_new)/X_z_new2 - (2*r33*X_x_new^2)/X_z_new3 - (2*r33*X_y_new^2)/X_z_new3) + 2*kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)*((2*r13*X_x_new)/X_z_new2 + (2*r23*X_y_new)/X_z_new2 - (2*r33*X_x_new^2)/X_z_new3 - (2*r33*X_y_new^2)/X_z_new3))*X_y_new)/X_z_new + (r23*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new + (2*kc4*r23*X_x_new)/X_z_new2 + (2*kc4*r13*X_y_new)/X_z_new2 - (r33*X_y_new*(kc1*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2) + kc2*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^2 + kc5*(X_x_new^2/X_z_new2 + X_y_new^2/X_z_new2)^3 + 1))/X_z_new2 - (4*kc4*r33*X_x_new*X_y_new)/X_z_new3);
            
        %for right image
            fx = F_right(1,j);
            fy = F_right(2,j);
            kc1 = KC_right(1,j);
            kc2 = KC_right(2,j);
            kc3 = KC_right(3,j);
            kc4 = KC_right(4,j);
            kc5 = KC_right(5,j);
            R11 = RT(1,1);R12 = RT(1,2);R13 = RT(1,3);
            R21 = RT(2,1);R22 = RT(2,2);R23 = RT(2,3);
            R31 = RT(3,1);R32 = RT(3,2);R33 = RT(3,3);
            T1 = RT(1,4);T2 = RT(2,4);T3 = RT(3,4);
            %The following sustitution is made for less computation
            r1x = t1 + r11*x + r12*y + r13*z;
            r2x = t2 + r21*x + r22*y + r23*z;
            r3x = t3 + r31*x + r32*y + r33*z;
            R1rx = T1 + R11*r1x + R12*r2x + R13*r3x;
            R2rx = T2 + R21*r1x + R22*r2x + R23*r3x;
            R3rx = T3 + R31*r1x + R32*r2x + R33*r3x;
            R1r1 = R11*r11 + R12*r21 + R13*r31;
            R1r2 = R11*r12 + R12*r22 + R13*r32;
            R1r3 = R11*r13 + R12*r23 + R13*r33;
            R2r1 = R21*r11 + R22*r21 + R23*r31;
            R2r2 = R21*r12 + R22*r22 + R23*r32;
            R2r3 = R21*r13 + R22*r23 + R23*r33;
            R3r1 = R31*r11 + R32*r21 + R33*r31;
            R3r2 = R31*r12 + R32*r22 + R33*r32;
            R3r3 = R31*r13 + R32*r23 + R33*r33;
            %The following result is got from jacobian. No need care for the
            %detail.
            B{i,j}(3,1) = fx*(kc4*((6*R1r1*(R1rx))/(R3rx)^2 + (2*R2r1*(R2rx))/(R3rx)^2 - (6*R3r1*(R1rx)^2)/(R3rx)^3 - (2*R3r1*(R2rx)^2)/R3rx^3) + (R1r1*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2) + 1))/(R3rx) + ((kc1*((2*R1r1*(R1rx))/(R3rx)^2 + (2*R2r1*(R2rx))/(R3rx)^2 - (2*R3r1*(R1rx)^2)/(R3rx)^3 - (2*R3r1*(R2rx)^2)/(R3rx)^3) + 2*kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)*((2*R1r1*(R1rx))/(R3rx)^2 + (2*R2r1*(R2rx))/(R3rx)^2 - (2*R3r1*(R1rx)^2)/(R3rx)^3 - (2*R3r1*(R2rx)^2)/(R3rx)^3) + 3*kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2*((2*R1r1*(R1rx))/(R3rx)^2 + (2*R2r1*(R2rx))/R3rx^2 - (2*R3r1*(R1rx)^2)/(R3rx)^3 - (2*R3r1*(R2rx)^2)/(R3rx)^3))*(R1rx))/(R3rx) - (R3r1*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/R3rx^2 + (R2rx)^2/(R3rx)^2) + 1)*(R1rx))/(R3rx)^2 + (2*kc3*R2r1*(R1rx))/(R3rx)^2 + (2*kc3*R1r1*(R2rx))/(R3rx)^2 - (4*kc3*R3r1*(R1rx)*(R2rx))/(R3rx)^3);
            B{i,j}(3,2) = fx*(kc4*((6*R1r2*(R1rx))/(R3rx)^2 + (2*R2r2*(R2rx))/(R3rx)^2 - (6*R3r2*(R1rx)^2)/(R3rx)^3 - (2*R3r2*(R2rx)^2)/R3rx^3) + (R1r2*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2) + 1))/(R3rx) + ((kc1*((2*R1r2*(R1rx))/(R3rx)^2 + (2*R2r2*(R2rx))/(R3rx)^2 - (2*R3r2*(R1rx)^2)/(R3rx)^3 - (2*R3r2*(R2rx)^2)/(R3rx)^3) + 2*kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)*((2*R1r2*(R1rx))/(R3rx)^2 + (2*R2r2*(R2rx))/(R3rx)^2 - (2*R3r2*(R1rx)^2)/(R3rx)^3 - (2*R3r2*(R2rx)^2)/(R3rx)^3) + 3*kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2*((2*R1r2*(R1rx))/(R3rx)^2 + (2*R2r2*(R2rx))/R3rx^2 - (2*R3r2*(R1rx)^2)/(R3rx)^3 - (2*R3r2*(R2rx)^2)/(R3rx)^3))*(R1rx))/(R3rx) - (R3r2*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/R3rx^2 + (R2rx)^2/(R3rx)^2) + 1)*(R1rx))/(R3rx)^2 + (2*kc3*R2r2*(R1rx))/(R3rx)^2 + (2*kc3*R1r2*(R2rx))/(R3rx)^2 - (4*kc3*R3r2*(R1rx)*(R2rx))/(R3rx)^3);
            B{i,j}(3,3) = fx*(kc4*((6*R1r3*(R1rx))/(R3rx)^2 + (2*R2r3*(R2rx))/(R3rx)^2 - (6*R3r3*(R1rx)^2)/(R3rx)^3 - (2*R3r3*(R2rx)^2)/R3rx^3) + (R1r3*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2) + 1))/(R3rx) + ((kc1*((2*R1r3*(R1rx))/(R3rx)^2 + (2*R2r3*(R2rx))/(R3rx)^2 - (2*R3r3*(R1rx)^2)/(R3rx)^3 - (2*R3r3*(R2rx)^2)/(R3rx)^3) + 2*kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)*((2*R1r3*(R1rx))/(R3rx)^2 + (2*R2r3*(R2rx))/(R3rx)^2 - (2*R3r3*(R1rx)^2)/(R3rx)^3 - (2*R3r3*(R2rx)^2)/(R3rx)^3) + 3*kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2*((2*R1r3*(R1rx))/(R3rx)^2 + (2*R2r3*(R2rx))/R3rx^2 - (2*R3r3*(R1rx)^2)/(R3rx)^3 - (2*R3r3*(R2rx)^2)/(R3rx)^3))*(R1rx))/(R3rx) - (R3r3*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/R3rx^2 + (R2rx)^2/(R3rx)^2) + 1)*(R1rx))/(R3rx)^2 + (2*kc3*R2r3*(R1rx))/(R3rx)^2 + (2*kc3*R1r3*(R2rx))/(R3rx)^2 - (4*kc3*R3r3*(R1rx)*(R2rx))/(R3rx)^3);
            B{i,j}(4,1) = fy*(kc3*((2*R1r1*(R1rx))/(R3rx)^2 + (6*R2r1*(R2rx))/(R3rx)^2 - (2*R3r1*(R1rx)^2)/(R3rx)^3 - (6*R3r1*(R2rx)^2)/R3rx^3) + (R2r1*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2) + 1))/(R3rx) + ((kc1*((2*R1r1*(R1rx))/(R3rx)^2 + (2*R2r1*(R2rx))/(R3rx)^2 - (2*R3r1*(R1rx)^2)/(R3rx)^3 - (2*R3r1*(R2rx)^2)/(R3rx)^3) + 2*kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)*((2*R1r1*(R1rx))/(R3rx)^2 + (2*R2r1*(R2rx))/(R3rx)^2 - (2*R3r1*(R1rx)^2)/(R3rx)^3 - (2*R3r1*(R2rx)^2)/(R3rx)^3) + 3*kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2*((2*R1r1*(R1rx))/(R3rx)^2 + (2*R2r1*(R2rx))/R3rx^2 - (2*R3r1*(R1rx)^2)/(R3rx)^3 - (2*R3r1*(R2rx)^2)/(R3rx)^3))*(R2rx))/(R3rx) - (R3r1*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/R3rx^2 + (R2rx)^2/(R3rx)^2) + 1)*(R2rx))/(R3rx)^2 + (2*kc4*R2r1*(R1rx))/(R3rx)^2 + (2*kc4*R1r1*(R2rx))/(R3rx)^2 - (4*kc4*R3r1*(R1rx)*(R2rx))/(R3rx)^3);
            B{i,j}(4,2) = fy*(kc3*((2*R1r2*(R1rx))/(R3rx)^2 + (6*R2r2*(R2rx))/(R3rx)^2 - (2*R3r2*(R1rx)^2)/(R3rx)^3 - (6*R3r2*(R2rx)^2)/R3rx^3) + (R2r2*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2) + 1))/(R3rx) + ((kc1*((2*R1r2*(R1rx))/(R3rx)^2 + (2*R2r2*(R2rx))/(R3rx)^2 - (2*R3r2*(R1rx)^2)/(R3rx)^3 - (2*R3r2*(R2rx)^2)/(R3rx)^3) + 2*kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)*((2*R1r2*(R1rx))/(R3rx)^2 + (2*R2r2*(R2rx))/(R3rx)^2 - (2*R3r2*(R1rx)^2)/(R3rx)^3 - (2*R3r2*(R2rx)^2)/(R3rx)^3) + 3*kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2*((2*R1r2*(R1rx))/(R3rx)^2 + (2*R2r2*(R2rx))/R3rx^2 - (2*R3r2*(R1rx)^2)/(R3rx)^3 - (2*R3r2*(R2rx)^2)/(R3rx)^3))*(R2rx))/(R3rx) - (R3r2*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/R3rx^2 + (R2rx)^2/(R3rx)^2) + 1)*(R2rx))/(R3rx)^2 + (2*kc4*R2r2*(R1rx))/(R3rx)^2 + (2*kc4*R1r2*(R2rx))/(R3rx)^2 - (4*kc4*R3r2*(R1rx)*(R2rx))/(R3rx)^3);
            B{i,j}(4,3) = fy*(kc3*((2*R1r3*(R1rx))/(R3rx)^2 + (6*R2r3*(R2rx))/(R3rx)^2 - (2*R3r3*(R1rx)^2)/(R3rx)^3 - (6*R3r3*(R2rx)^2)/R3rx^3) + (R2r3*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2) + 1))/(R3rx) + ((kc1*((2*R1r3*(R1rx))/(R3rx)^2 + (2*R2r3*(R2rx))/(R3rx)^2 - (2*R3r3*(R1rx)^2)/(R3rx)^3 - (2*R3r3*(R2rx)^2)/(R3rx)^3) + 2*kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)*((2*R1r3*(R1rx))/(R3rx)^2 + (2*R2r3*(R2rx))/(R3rx)^2 - (2*R3r3*(R1rx)^2)/(R3rx)^3 - (2*R3r3*(R2rx)^2)/(R3rx)^3) + 3*kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2*((2*R1r3*(R1rx))/(R3rx)^2 + (2*R2r3*(R2rx))/R3rx^2 - (2*R3r3*(R1rx)^2)/(R3rx)^3 - (2*R3r3*(R2rx)^2)/(R3rx)^3))*(R2rx))/(R3rx) - (R3r3*(kc2*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^2 + kc5*((R1rx)^2/(R3rx)^2 + (R2rx)^2/(R3rx)^2)^3 + kc1*((R1rx)^2/R3rx^2 + (R2rx)^2/(R3rx)^2) + 1)*(R2rx))/(R3rx)^2 + (2*kc4*R2r3*(R1rx))/(R3rx)^2 + (2*kc4*R1r3*(R2rx))/(R3rx)^2 - (4*kc4*R3r3*(R1rx)*(R2rx))/(R3rx)^3);
        
        end
    end
end

%eps is the reprojection error
eps = cell(N,M);
for i = 1:N
    for j = 1:M
        if delta(i,j)==1
            eps{i,j}(1:2,1) = -(x_p{1,j}(:,i) - I_left{j}(:,i));
            eps{i,j}(3:4,1) = -(x_p{2,j}(:,i) - I_right{j}(:,i));
        else
            eps{i,j} = zeros(4,1);
        end
    end
end

LM_righthand_a = cell(M,1);
for j = 1:M
    LM_righthand_a{j} = 0;
    for i = 1:N
        LM_righthand_a{j} = LM_righthand_a{j} + A{i,j}'*eps{i,j};
    end
end

LM_righthand_b = cell(N,1);
for i = 1:N
    LM_righthand_b{i} = 0;
    for j = 1:M
        LM_righthand_b{i} = LM_righthand_b{i} + B{i,j}'* eps{i,j};
    end
end


U = cell(M);
for j = 1:M
    U{j,j} = 0;
    for i = 1:N
        U{j,j} = U{j,j} + A{i,j}'*A{i,j};
    end
end

for j = 1:M*M
    if length(U{j})==0
        U{j} = zeros(size(U{1}));
    end
end

V = cell(N);
for i = 1:N
    V{i,i} = 0;
    for j = 1:M
        V{i,i} = V{i,i}+B{i,j}'*B{i,j};
    end
end

for i = 1:N*N
    if length(V{i}) == 0
        V{i} = zeros(size(V{1}));
    end
end
        
W = cell(M,N);
for j = 1:M
    for i = 1:N
        W{j,i} = A{i,j}'*B{i,j};
    end
end
        
%LM_lefthand = {U W; W' V};
U_mat = cell2mat(U)+miu*eye(size(cell2mat(U)));
V_mat = cell2mat(V)+miu*eye(size(cell2mat(V)));
W_mat = cell2mat(W);

%LM_lefthand_mat = [U_mat W_mat; W_mat' V_mat];
LM_righthand_a_mat = cell2mat(LM_righthand_a);
LM_righthand_b_mat = cell2mat(LM_righthand_b);

delta_a = (U_mat-W_mat/(V_mat)*W_mat')\(LM_righthand_a_mat - W_mat/(V_mat)*LM_righthand_b_mat);
delta_b = (V_mat)\(LM_righthand_b_mat-W_mat'*delta_a);

delta_a = reshape(delta_a, size(A{1},2), M);
delta_b = reshape(delta_b, size(B{1},2), N);

S0_new = S0 + delta_b;

P_new = cell(1,M);

for j = 1:M    
    R = rodrigues(rodrigues(P{j}(:,1:3))+delta_a(1:3,j));
    T = P{j}(:,4)+delta_a(4:6,j);
    P_new{j}= [R,T];
end

F_new = zeros(4,M);
C_new = zeros(4,M);
KC_new = zeros(10,M);

F_new(1:2,:) = F_left + delta_a(7:8,:);
C_new(1:2,:) = C_left + delta_a(9:10,:);
KC_new(1:5,:) = KC_left + delta_a(11:15,:);

F_new(3:4,:) = F_right + delta_a(16:17,:);
C_new(3:4,:) = C_right + delta_a(18:19,:);
KC_new(6:10,:) = KC_right + delta_a(20:24,:);

avg_distance = get_E(S0_new,P_new, I_left, I_right, ref_global);
        
        
        
        
        
        
        
        
        
        
        
        
        
