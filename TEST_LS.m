% TEST Least-Squares Controller
%   Code written to test the least-squares controller
clear all; close all; clc;
%% Control Points

%In Camera frame of lead drone
P = [-1 -1 -1;
     -1  1 -1;
      1 -1 -1;
      1  1 -1];

% Given states of both drones
            %[X  Y  Z   roll        pitch       yaw]
drone_lead = [30 10 -4  0           0              0];
drone_folw = [25 8 -8   5*pi/180    -3*pi/180      2*pi/180];

M = 1000;N=1000;    %pixel dimensions
H = 0.05;W=0.05;    %image plane dimensions

cam.fx = .005*M/H;
cam.fy = .005*N/W;
cam.gamma = 0;
cam.cx = M/2;
cam.cy = N/2;

[Q t ang depth] = generate_image_points(drone_lead,drone_folw,cam,P);

if or(Q(:,1) > M, Q(:,2) > N)
    error('Lead vehicle is outside image dimensions')
else
    
    %Add noise to input
    x_n = [ang+randn/10, t'+2*randn(1,3)];
    
    [Rx tx] = estimate_pose(x_n,cam,P,Q);
    
    disp('TRUE')
    disp(sprintf('Translation vector: [%3.3f %3.3f %3.3f]',t));
    disp(sprintf('Roll: %.3f \t Pitch: %.3f \t Yaw: %.3f',ang*180/pi));
    disp('ESTIMATED')
    disp(sprintf('Translation vector: [%3.3f %3.3f %3.3f]',tx));
    disp(sprintf('Roll: %.3f \t Pitch: %.3f \t Yaw: %.3f',Rx*180/pi));
    disp('ERROR')
    disp(sprintf('Translation vector: [%3.3f %3.3f %3.3f]',t-tx'));
    disp(sprintf('Roll: %.3f \t Pitch: %.3f \t Yaw: %.3f',(ang-Rx)*180/pi));
    
end

% x_n = [x_n(1:3)*180/pi,x_n(4:6)];
% 
% x_est = [Rx*180/pi, t];

% hold on
% plot3(P(:,1),P(:,2),P(:,3),'*'),grid,axis equal
% plot3([0 t(1)],[0 t(2)],[0 t(3)],'r')
% plot3(Q(:,1)+t(1),Q(:,2)+t(2),Q(:,3)*t(3)-f,'*r')
% quiver3(t(1)*ones(1,3),t(2)*ones(1,3),t(3)*ones(1,3),R(:,1)',R(:,2)',R(:,3)',0,'r')
% quiver3(zeros(1,3),zeros(1,3),zeros(1,3),[1,0,0],[0,1,0],[0,0,1],0,'b')
% hold off