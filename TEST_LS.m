% TEST Least-Squares Controller
%   Code written to test the least-squares controller
clear all; close all; clc;
%% Control Points

P = [-1 -1 -1;
     -1  1 -1;
      1 -1 -1;
      1  1 -1];


%% Compute the known s^c vector

roll = 10*pi/180;
pitch = 0;
yaw = -6*pi/180;

%Find DCM from b1 to b2

R = [cos(pitch)*cos(yaw), -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
      cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
      -sin(pitch)          , sin(roll)*cos(pitch),          cos(roll)*cos(pitch)];
  
%Given translation vector from origin c2 to origin c1

t = [3 -2 -10];

sC = R*P(1:4,:)' - [t',t',t',t'];

%Project onto image plane

f = 1;
for i = 1:length(sC)
   pts = [f 0 0;
          0 f 0;
          0 0 1]*sC(:,i)/sC(3,i);
   Q(i,:) = pts;    
end

x_n = [roll+rand/10, pitch+rand/10, yaw+rand/10, t+rand(1,3)];

[Rx t] = estimate_pose(x_n,f,P,Q);

x_n = [x_n(1:3)*180/pi,x_n(4:6)];

x_est = [Rx*180/pi, t];

hold on
plot3(P(:,1),P(:,2),P(:,3),'*'),grid,axis equal
plot3([0 t(1)],[0 t(2)],[0 t(3)],'r')
plot3(Q(:,1)+t(1),Q(:,2)+t(2),Q(:,3)*t(3)-f,'*r')
quiver3(t(1)*ones(1,3),t(2)*ones(1,3),t(3)*ones(1,3),R(:,1)',R(:,2)',R(:,3)',0,'r')
quiver3(zeros(1,3),zeros(1,3),zeros(1,3),[1,0,0],[0,1,0],[0,0,1],0,'b')
hold off