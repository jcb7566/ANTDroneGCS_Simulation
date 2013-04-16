function dF = implicit_equation(x_n,f,P,Q)
% dF = implicit_equation(x_n,f,P,Q)
% Compute the implicit function [f(x);g(x)] = [0;0]
% Input:    x_n: nominal state (by differencing lead and follower states
%           f: focal point of camera calibration matrix
%           P: matrix of control points on lead drone [x,y,z]
%           Q: imaged control points in camera of follower [u,v]
% Output:   dF: a vector of the implicit function for each point pair

dF = zeros(length(Q),1);

R = [cos(x_n(2))*cos(x_n(3)), -cos(x_n(1))*sin(x_n(3)) + sin(x_n(1))*sin(x_n(2))*cos(x_n(3)), sin(x_n(1))*sin(x_n(3)) + cos(x_n(1))*sin(x_n(2))*cos(x_n(3));
      cos(x_n(2))*sin(x_n(3)), cos(x_n(1))*cos(x_n(3)) + sin(x_n(1))*sin(x_n(2))*sin(x_n(3)), -sin(x_n(1))*cos(x_n(3)) + cos(x_n(1))*sin(x_n(2))*sin(x_n(3));
      -sin(x_n(2))          , sin(x_n(1))*cos(x_n(2)),          cos(x_n(1))*cos(x_n(2))];                                    



for i = 1:length(Q)
    A = [f*P(i,:), 0,0,0, -Q(i,1).*P(i,:), f, 0, -Q(i,1);
         0,0,0, f*P(i,:), -Q(i,2).*P(i,:), f, 0, -Q(i,2)]; 
    b = [R(1,1), R(1,2), R(1,3), R(2,1), R(2,2), R(2,3), R(3,1), R(3,2), R(3,3), x_n(4), x_n(5), x_n(6)]';
    
    dF(2*i-1:2*i) = A*b;

end
    
end