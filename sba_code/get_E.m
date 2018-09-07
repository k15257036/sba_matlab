function avg_distance = get_E(S0,P, I_left, I_right, ref_global, filename)
%   avg_distance = get_E(S0, P, I_left, I_right, ref_global, filename)
%
%           Calculate the avarage distance between the reprojection pixel
%           and the original in the image in the both left and right
%           cameras.
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

if nargin < 6
    filename = 'CameraCalibration.xml';
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
        disp('the point''s number of image: ');
        disp(j)
        disp('is not equal between the left and right');
        return;
    end
end


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
[F_left,C_left,KC_left,F_right,C_right,KC_right,RT] = read_calibration_xml(filename);

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
%
%For right image, from S0 to xij:  R*(P_R*S0+P_T)+T -> xij
%       P_om_right = rodrigues(R*P_R)
%       P_T_right = R*P_T+T
%       <-> project_points2(S0,om_right,T_right)
[P_om_right,P_T_right] = deal(zeros(3,M));
for j = 1:M
    P_om_right(:,j) = rodrigues(RT(:,1:3)*P{j}(:,1:3));
    P_T_right(:,j) = RT(:,1:3)*P{j}(:,4)+RT(:,4);
end

delta = logical(ref_global);

%x_p is the reprojection 2d points of each image.
x_p = cell(2,M);
for j = 1:M
    x_p{1,j} = project_points2(S0,P_om(:,j),P_T(:,j),F_left,C_left,KC_left);
    x_p{1,j} = x_p{1,j}.*delta(:,j)';
    x_p{2,j} = project_points2(S0,P_om_right(:,j),P_T_right(:,j),F_right,C_right,KC_right);
    x_p{2,j} = x_p{2,j}.*delta(:,j)';
end


avg_distance = 0;
total = 2*sum(sum(delta));

for j = 1:M
        avg_distance = avg_distance + sum(sum(abs(x_p{1,j} - I_left{j})));
        avg_distance = avg_distance + sum(sum(abs(x_p{2,j} - I_right{j})));
end

avg_distance = avg_distance/total;

