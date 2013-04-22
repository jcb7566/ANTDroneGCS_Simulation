function dF = implicit_equation(roll,pitch,yaw,t,cam,P,Q)
% dF = implicit_equation(x_n,f,P,Q)
% Compute the implicit function [f(x);g(x)] = [0;0]
% Input:    roll,pitch,yaw: euler angles between lead nad follower drone
%           states
%           t: translation vector between cameras
%           cam: calibration parameters of camera
%           P: matrix of control points on lead drone [x,y,z] in camera frame
%           Q: imaged control points in camera of follower [u,v] in camera
%           frame
% Output:   dF: a vector of the implicit function for each point pair

dF = zeros(length(Q),1);

C = [cos(pitch)*cos(yaw), -cos(roll)*sin(pitch) + sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
      cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
      -sin(pitch)          , sin(roll)*cos(pitch),          cos(roll)*cos(pitch)];
T = [0 1 0;0 0 1;1 0 0];

R = T*C*T';

for i = 1:length(Q)
    A = [cam.fx*P(i,:), cam.gamma*P(i,:), (cam.cx-Q(i,1)).*P(i,:), cam.fx, cam.gamma, cam.cx-Q(i,1);
         0,0,0,        cam.fy*P(i,:),     (cam.cy-Q(i,2)).*P(i,:), 0,     cam.fy,         cam.cy-Q(i,2)]; 
    b = [R(1,1), R(1,2), R(1,3), R(2,1), R(2,2), R(2,3), R(3,1), R(3,2), R(3,3), t]';
    
    dF(2*i-1:2*i) = A*b;

end
    
end