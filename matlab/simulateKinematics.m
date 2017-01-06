function [T,X] = simulateKinematics(X0,tf,goalPose, v,omega)

% Differential equations that determine state
dxdt =@(t,x) [v(x,goalPose)*cos(x(3)); ...   % dx/dt
              v(x,goalPose)*sin(x(3)); ...   % dy/dt
              omega(x,goalPose)];            % dtheta/dt

% Timing (in seconds)
ti=0; tspan=[ti tf];

% Use ode45 to solve
[T,X]=ode23(dxdt,tspan,X0);
end