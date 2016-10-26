% This code creates  obstacle at some random point in front of the robot.
% It then calculates the radius of the constant curvature path that
% intersects with that point. Finally, it plots the obstacle and the path.

% Create 'num_obs' obstacles at random x,y locations in front of the robot.
num_obs = 5;
xmax = 10;
ymax = 10;
x_obs = 2*xmax*rand(num_obs,1)-xmax;
y_obs = ymax*rand(num_obs,1);

% Calculate the radius of curvature of a circle that would pass through
% those points.
R = (x_obs.^2+y_obs.^2)./(2*x_obs);

% Plot the obstacles and the paths to reach them
figure(1)
for i = 1:num_obs
    x = linspace(0,x_obs(i),100);
    y = abs(sqrt(2*R(i)-x).*sqrt(x));
    plot(x_obs,y_obs,'xr',x,y)
    hold on
end
hold off
axis equal
xlim([-xmax xmax])
ylim([0 ymax])