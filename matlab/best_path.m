% This code creates  obstacle at some random point in front of the robot.
% It then calculates the radius of the constant curvature path that
% intersects with that point. Finally, it plots the obstacle and the path.

%% Create 'num_obs' obstacles at random x,y locations in front of the robot.
num_obs = 1;
xmax = 10;
ymax = 10;
x_obs = xmax*rand(num_obs,1);
y_obs = 2*ymax*rand(num_obs,1)-ymax;

%% Define vehicle (max width and speed)
% Max width of vehicle
max_width = 1;
half_width = max_width/2;
% Commanded linear and angular velocity pre-obstacle avoidance
v_pre = 2;
omega_pre = .1;
R_pre = v_pre/omega_pre;

%% Calculate ideal omega for obstacle avoidance
% Calculate the radius of curvature of a circle that would just touch those
% points on the left or right side. R_right means that the robot passes the
% obstacle with the obstacle on the right, and opposite for R_left.
R_left = ((x_obs).^2+(y_obs+half_width).^2)./(2*(y_obs+half_width))-half_width;
R_right = ((x_obs).^2+(y_obs-half_width).^2)./(2*(y_obs-half_width))+half_width;
R_pos = [R_left; R_right];
% Possible values of omega
omega_pos = v_pre./R_pos;

omega_max_left = min(omega_pos(omega_pos-omega_pre<0));
omega_max_right = max(omega_pos(omega_pos-omega_pre>0));

% Of the possible omegas, we want to choose the best according to the
% following criteria:
% 1.) If there are no obstacles in the current path, keep the current omega
% 2.) If there are 1 or more obstacles in the path
omega_new = omega_pos(min(abs(omega_pos-omega_pre))==abs(omega_pos-omega_pre));
delta_omega = omega_new-omega_pre;
R_new = R_pos(min(abs(omega_pos-omega_pre))==abs(omega_pos-omega_pre));

%% Plot the obstacles and the paths to reach them
figure(1)
cols = get(gca,'colororder');
% Plot the original path in black
y_mid = 0:sign(R_pre)*0.1:2*R_pre;
y_left = -half_width:sign(R_pre+half_width-(-half_width))*0.1:2*R_pre+half_width;
y_right = half_width:sign(R_pre+half_width-(-half_width))*0.1:2*R_pre-half_width;

x_mid = abs(sqrt(2*R_pre-y_mid).* sqrt(y_mid));
x_left  = abs(sqrt(half_width+2*R_pre-y_left ).*sqrt(half_width+y_left ));
x_right = abs(sqrt(half_width-2*R_pre+y_right).*sqrt(half_width-y_right));

plot(x_mid,y_mid,'--','Color',[0 0 0])
hold on
plot(x_left,y_left,'Color',[0 0 0])
plot(x_right,y_right,'Color',[0 0 0])

for i = 1:num_obs
    y_mid = 0:sign(R_left(i)+half_width-(-half_width))*0.1:2*R_left(i);
    y_left = -half_width:sign(R_left(i)+half_width-(-half_width))*0.1:2*R_left(i)+half_width;
    y_right = half_width:sign(R_left(i)+half_width-(-half_width))*0.1:2*R_left(i)-half_width;
    
    x_mid   = abs( sqrt(              2*R_left(i) - y_mid   ) .* sqrt(              y_mid   ));
    x_left  = abs( sqrt( half_width + 2*R_left(i) - y_left  ) .* sqrt( half_width + y_left  ));
    x_right = abs( sqrt( half_width - 2*R_left(i) + y_right ) .* sqrt( half_width - y_right ));
    
    plot(x_obs(i),y_obs(i),'xr','MarkerSize',10,'LineWidth',2)
    plot(x_mid,y_mid,'--','Color',cols(mod(i,7)+1,:))
    plot(x_left,y_left,'Color',cols(mod(i,7)+1,:))
    plot(x_right,y_right,'Color',cols(mod(i,7)+1,:))
end
for i = 1:num_obs
    y_mid = 0:sign(R_right(i)+half_width-(-half_width))*0.1:2*R_right(i);
    y_left = -half_width:sign(R_right(i)+half_width-(-half_width))*0.1:2*R_right(i)+half_width;
    y_right = half_width:sign(R_right(i)+half_width-(-half_width))*0.1:2*R_right(i)-half_width;
    
    x_mid   = abs( sqrt(              2*R_right(i) - y_mid   ) .* sqrt(              y_mid   ));
    x_left  = abs( sqrt( half_width + 2*R_right(i) - y_left  ) .* sqrt( half_width + y_left  ));
    x_right = abs( sqrt( half_width - 2*R_right(i) + y_right ) .* sqrt( half_width - y_right ));
    
    plot(x_obs(i),y_obs(i),'xr','MarkerSize',10,'LineWidth',2)
    plot(x_mid,y_mid,'--','Color',cols(mod(i+num_obs,7)+1,:))
    plot(x_left,y_left,'Color',cols(mod(i+num_obs,7)+1,:))
    plot(x_right,y_right,'Color',cols(mod(i+num_obs,7)+1,:))
end
hold off
axis equal
xlim([0 xmax])
ylim([-ymax ymax])