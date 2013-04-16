function [R t] = estimate_pose(x_n,focal_point, control_points,image_points)
% [R t] = estimate_pose(focal_point, control_points,image_points)
% Estimate translation and rotation of a camera viewing known viewpoints in
% a scene using least square technique. ** could be converted to EKF
% Input:    focal_point: focal point of the camera calibration matrix
%           (follower)
%           control_points: known target points (coplanar)
%           image_points: projective coordinates of control points in body
%           frame of lead drone
% Output:   R: rotation DCM, from camera frame to target frame
%           t: translational vector, expressed in camera frame

while(1)
    clc
%1. Compute f(x_n) and g(x_n)
dF = implicit_equation(x_n,focal_point,control_points,image_points)
%2. Compute H
H = H_least_squares(x_n,focal_point,control_points,image_points);
%3. Iterate least squares
dx = (H'*H)\H'*dF;

    %check for convergence
    if abs(dx) < [10e-7*ones(1,3),10e-5*ones(1,3)]'
        break;
    end

%4. Compute new nominal
x_n = x_n - dx'
end

R = x_n(1:3);
t = x_n(4:6);

end