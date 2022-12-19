function x = myStateTransitionFcn(x,u)
% Sample time [s]
Ts = 0.01; 
m = 1;
L = 0.5;  
g = 9.81; 

% Write your code here to define the discrete-time state update equations
% you've found in Part3-task15 of the Kalman filter virtual lab 
% tmp = x(1);
% x(1) = Ts*x(2)+x(1);
% x(2) = Ts*(-g/l*sin(tmp)+u/(m*L^2))+x(2);

x = x + Ts*[x(2); -g/L*sin(x(1))+u/(m*L^2)];

end

% Copyright 2022 The MathWorks, Inc. 