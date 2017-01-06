function [T,X] = simulateKinematics(X0,tf,v,omega)

% Differential equations that determine state
dxdt =@(t,x) [v(x)*cos(x(3)); ...   % dx/dt
              v(x)*sin(x(3)); ...   % dy/dt
              omega(x)];            % dtheta/dt

% Timing (in seconds)
ti=0; tspan=[ti tf];

% Use ode45 to solve
[T,X]=ode23(dxdt,tspan,X0);
end