function [T,X] = simulateKinematics(X0,tf,dvdt,dwdt)

% Differential equations that determine state
dxdt =@(t,x) [x(4)*cos(x(3)); ...   % dx/dt
              x(4)*sin(x(3)); ...   % dy/dt
              x(5); ...             % dtheta/dt
              dvdt(x); ...                % dv/dt
              dwdt(x)];                   % dw/dt
% Timing (in seconds)
ti=0; tspan=[ti tf];

% Use ode45 to solve
[T,X]=ode23(dxdt,tspan,X0);
end