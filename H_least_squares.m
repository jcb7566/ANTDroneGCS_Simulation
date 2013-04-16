function H = H_least_squares(x_n, f, P, Q)

H = zeros(2*length(P),6);

for i = 1:length(P)
    %Partials of f(x)
    H(2*i-1,1) = f*P(i,2)*(sin(x_n(1))*sin(x_n(3)) + cos(x_n(1))*sin(x_n(2))*cos(x_n(3))) + f*P(i,3)*(cos(x_n(1))*sin(x_n(3)) - ...
                sin(x_n(1))*sin(x_n(2))*cos(x_n(3)))-Q(i,1)*P(i,2)*(cos(x_n(1))*cos(x_n(2)))+Q(i,1)*P(i,3)*(sin(x_n(1))*cos(x_n(2)));
    H(2*i-1,2) = -f*P(i,1)*(sin(x_n(2))*cos(x_n(3))) + f*P(i,2)*(sin(x_n(1))*cos(x_n(2))*cos(x_n(3))) + f*P(i,3)*(cos(x_n(1))*cos(x_n(2))*cos(x_n(3))) + ...
                Q(i,1)*P(i,1)*cos(x_n(2)) + Q(i,1)*P(i,2)*(sin(x_n(1))*sin(x_n(2))) + Q(i,1)*P(i,3)*(cos(x_n(1))*sin(x_n(2)));
    H(2*i-1,3) = -f*P(i,1)*(sin(x_n(2))*cos(x_n(3))) - f*P(i,2)*(cos(x_n(1))*cos(x_n(3)) + sin(x_n(1))*sin(x_n(2))*sin(x_n(3))) + ...
                 f*P(i,3)*(sin(x_n(1))*cos(x_n(3)) - cos(x_n(1))*sin(x_n(2))*sin(x_n(3)));
    H(2*i-1,4) = f;
    H(2*i-1,6) = -Q(i,1);
    
    %Partials of g(x)
    H(2*i,1) = f*P(i,2)*(-sin(x_n(1))*cos(x_n(3)) + cos(x_n(1))*sin(x_n(2))*sin(x_n(3))) - f*P(i,3)*(cos(x_n(1))*cos(x_n(3)) + ...
                sin(x_n(1))*sin(x_n(2))*sin(x_n(3)))-Q(i,2)*P(i,2)*(cos(x_n(1))*cos(x_n(2)))+Q(i,2)*P(i,3)*(sin(x_n(1))*cos(x_n(2)));
    H(2*i,2) = -f*P(i,1)*(sin(x_n(2))*sin(x_n(3))) + f*P(i,2)*(sin(x_n(1))*cos(x_n(2))*sin(x_n(3))) + f*P(i,3)*(cos(x_n(1))*cos(x_n(2))*sin(x_n(3))) + ...
                Q(i,2)*P(i,1)*cos(x_n(2)) + Q(i,2)*P(i,2)*(sin(x_n(1))*sin(x_n(2))) + Q(i,2)*P(i,3)*(cos(x_n(1))*sin(x_n(2)));
    H(2*i,3) = f*P(i,1)*(cos(x_n(2))*cos(x_n(3))) + f*P(i,2)*(-cos(x_n(1))*sin(x_n(3)) + sin(x_n(1))*sin(x_n(2))*cos(x_n(3))) + ...
                f*P(i,3)*(sin(x_n(1))*sin(x_n(3)) + cos(x_n(1))*sin(x_n(2))*cos(x_n(3)));
    H(2*i,5) = f;
    H(2*i,6) = -Q(i,2);

end