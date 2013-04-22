function H = H_least_squares(x_n, cam, P, Q)

fx= cam.fx;
fy= cam.fy;
cx = cam.cx;
cy = cam.cy;
g =cam.gamma;

H = zeros(2*length(P),6);

for i = 1:length(P)
    %Partials of f(x)
    H(2*i-1,1) = fx*P(i,1)*cos(x_n(1))*sin(x_n(2))*sin(x_n(3))-fx*P(i,2)*(cos(x_n(1))*cos(x_n(3))+sin(x_n(1))*sin(x_n(2))*sin(x_n(3)))+ ...
                 g*P(i,1)*cos(x_n(1))*cos(x_n(2))-g*P(i,2)*sin(x_n(1))*cos(x_n(2))+(cx-Q(i,1))*P(i,1)*(sin(x_n(1))*sin(x_n(3))+cos(x_n(1))*sin(x_n(2))*cos(x_n(3)))+ ...
                 (cx-Q(i,1))*P(i,2)*(cos(x_n(1))*sin(x_n(3))-sin(x_n(1))*sin(x_n(2))*cos(x_n(3)));
    H(2*i-1,2) = fx*P(i,1)*(-sin(x_n(2))*cos(x_n(3))+sin(x_n(1))*cos(x_n(2))*sin(x_n(3)))+fx*P(i,2)*cos(x_n(1))*cos(x_n(2))*sin(x_n(3))- ...
                 fx*P(i,3)*sin(x_n(2))*sin(x_n(3))-g*P(i,1)*sin(x_n(1))*sin(x_n(2))-g*P(i,2)*cos(x_n(1))*sin(x_n(2))-g*P(i,3)*cos(x_n(2))+ ...
                 (cx-Q(i,1))*P(i,1)*sin(x_n(1))*cos(x_n(2))*cos(x_n(3))+(cx-Q(i,1))*P(i,2)*cos(x_n(1))*cos(x_n(2))*cos(x_n(3))- ...
                 (cx-Q(i,1))*P(i,3)*sin(x_n(2))*cos(x_n(3));
    H(2*i-1,3) = fx*P(i,1)*(-cos(x_n(2))*sin(x_n(3))+sin(x_n(1))*sin(x_n(2))*cos(x_n(3)))+fx*P(i,2)*(sin(x_n(1))*sin(x_n(3))+cos(x_n(1))*sin(x_n(2))*cos(x_n(3)))+ ...
                 fx*P(i,3)*cos(x_n(2))*cos(x_n(3))-(cx-Q(i,1))*P(i,1)*(cos(x_n(1))*cos(x_n(3))+sin(x_n(1))*sin(x_n(2))*sin(x_n(3)))+ ...
                 (cx-Q(i,1))*P(i,2)*(sin(x_n(1))*cos(x_n(3))-cos(x_n(1))*sin(x_n(2))*sin(x_n(3)))-(cx-Q(i,1))*P(i,3)*cos(x_n(2))*sin(x_n(3));
    H(2*i-1,4) = fx;
    H(2*i-1,5) = g;
    H(2*i-1,6) = cx-Q(i,1);
    
    %Partials of g(x)
    H(2*i,1) = fy*P(i,1)*cos(x_n(1))*cos(x_n(2))-fy*P(i,2)*sin(x_n(1))*cos(x_n(2))+(cy-Q(i,2))*P(i,1)*(sin(x_n(1))*sin(x_n(3))+cos(x_n(1))*sin(x_n(2))*cos(x_n(3)))+ ...
               (cy-Q(i,2))*P(i,2)*(cos(x_n(1))*sin(x_n(3))-sin(x_n(1))*sin(x_n(2))*cos(x_n(3)));
    H(2*i,2) = -fy*P(i,1)*sin(x_n(1))*sin(x_n(2))-fy*P(i,2)*cos(x_n(1))*sin(x_n(2))-fy*P(i,3)*cos(x_n(2))+(cy-Q(i,2))*P(i,1)*sin(x_n(1))*cos(x_n(2))*cos(x_n(3))+ ...
               (cy-Q(i,2))*P(i,2)*cos(x_n(1))*cos(x_n(2))*cos(x_n(3))-(cy-Q(i,2))*P(i,3)*sin(x_n(2))*cos(x_n(3));
    H(2*i,3) = -(cy-Q(i,2))*P(i,1)*(cos(x_n(1))*cos(x_n(3))+sin(x_n(1))*sin(x_n(2))*sin(x_n(3)))+(cy-Q(i,2))*P(i,2)*(sin(x_n(1))*cos(x_n(3))-cos(x_n(1))*sin(x_n(2))*sin(x_n(3)))- ...
               (cy-Q(i,2))*P(i,3)*cos(x_n(2))*sin(x_n(3));
    H(2*i,5) = fy;
    H(2*i,6) = cy-Q(i,2);

end