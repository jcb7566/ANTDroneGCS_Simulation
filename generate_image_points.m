function [Q t ang Z] = generate_image_points(drone_lead,drone_folw,cam,P)


%find relative pose
drone_diff = drone_lead-drone_folw;
  
  
% Compute the s^c vector

roll = drone_diff(4);
pitch= drone_diff(5);
yaw = drone_diff(6);

ang = [roll pitch yaw];
%Find DCM from c1 to c2

C = [cos(pitch)*cos(yaw), -cos(roll)*sin(pitch) + sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
      cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
      -sin(pitch)          , sin(roll)*cos(pitch),          cos(roll)*cos(pitch)];
T = [0 1 0;0 0 1;1 0 0];

C_c1_c2 = T*C*T';

%Find DCM from nav to body
C_n_b = [cos(drone_folw(5))*cos(drone_folw(6)), -cos(drone_folw(4))*sin(drone_folw(6)) + sin(drone_folw(4))*sin(drone_folw(5))*cos(drone_folw(6)), sin(drone_folw(4))*sin(drone_folw(6)) + cos(drone_folw(4))*sin(drone_folw(5))*cos(drone_folw(6));
      cos(drone_folw(5))*sin(drone_folw(6)), cos(drone_folw(4))*cos(drone_folw(6)) + sin(drone_folw(4))*sin(drone_folw(5))*sin(drone_folw(6)), -sin(drone_folw(4))*cos(drone_folw(6)) + cos(drone_folw(4))*sin(drone_folw(5))*sin(drone_folw(6));
      -sin(drone_folw(5))          , sin(drone_folw(4))*cos(drone_folw(5)),          cos(drone_folw(4))*cos(drone_folw(5))]';
  
%Find translation vector from origin c2 to origin c1

t = [0 1 0;0 0 1;1 0 0]*C_n_b*drone_diff(1:3)';

sC = C_c1_c2*P(1:4,:)' + [t,t,t,t];
Z = sC(3,:);
%Project onto image plane
for i = 1:length(sC)
   pts = [cam.fx cam.gamma     cam.cx;
          0     cam.fy         cam.cy;
          0     0             1]*sC(:,i)/sC(3,i);
   Q(i,:) = pts+randn;    
end

end