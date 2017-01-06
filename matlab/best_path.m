% This code creates obstacles at random points in front of the robot.
% It then calculates the radius of the constant curvature path that
% intersects with that point. Finally, it plots the obstacle and the path.

%% Create 'num_obs' obstacles at random x,y locations in front of the robot.
num_obs = 10;
% Define the box in front of the robot (0:xmax,-ymax:ymax)
xmax = 10;
ymax = 10;
% store random x and y points of obstacles in two arrays
% _tot suffix is used to designate these as all teh obstacles
x_obs_tot = xmax*rand(num_obs,1);
y_obs_tot = 2*ymax*rand(num_obs,1)-ymax;

%% Define vehicle (max width and speed)
% Max width of vehicle
max_width = 1;
half_width = max_width/2;
% Commanded linear and angular velocity pre-obstacle avoidance
v_pre = 2;
omega_pre = .1;
% And the radius of curvature
R_pre = v_pre/omega_pre;

% Find which obstacles are in the path
% TODO: Add distance filter as well (only worry about close
% obstacles)
% The robot drives along a circle defined by R_pre. An inner and
% outer circle are offset from this by half_width.
in_path = @(x,y,r,w) sqrt(x.^2+(r-y).^2)>=(r-w/2) & ...
                     sqrt(x.^2+(r-y).^2)<=(r+w/2);
in_path_pre = in_path(x_obs_tot,y_obs_tot,R_pre,max_width);
x_obs = x_obs_tot(:);
y_obs = y_obs_tot(:);

% TODO: Once new path is found, see if an obstacle is in the new path. If a
% larger change is needed to avoid any new obstacles, see if turning the
% other direction is better. Keep checking until there are no obstacles in
% the new path.

%% Calculate ideal omega for obstacle avoidance
% Calculate the radius of curvature of a circle that would just touch those
% points on the left or right side. R_right means that the robot passes the
% obstacle to the right of the obstacle, and opposite for R_left.
R_left = ((x_obs).^2+(y_obs+half_width).^2)./(2*(y_obs+half_width))-half_width;
R_right = ((x_obs).^2+(y_obs-half_width).^2)./(2*(y_obs-half_width))+half_width;
R_pos = [R_left; R_right];
% Possible values of omega
omega_pos = v_pre./R_pos;

% Create a table of obstacle lable, x,y position, and left/right curvature.
% Sort left and right path's in order of increasing curvature. (1/R)
table = [(1:num_obs)' x_obs y_obs 1./R_left 1./R_right];
table_left = sortrows(table,4);
table_right = sortrows(table,5);

% Look left. Find the first obstacle we can pass to its left, with room
% between it and the next obstacle.
n = find(table_left(:,4)>1/R_pre,1);
dist = sqrt((table_left(n,2)-table_left(n+1,2))^2+(table_left(n,3)-table_left(n+1,3))^2);
while dist < max_width && n < num_obs
    n = n+1;
    dist = sqrt((table_left(n,2)-table_left(n+1,2))^2+(table_left(n,3)-table_left(n+1,3))^2);
end

curvLeft = table_left(n,4);

% Look right. Find the first obstacle we can pass to its right, with room
% between it and the next obstacle.
n = num_obs+1-find(flipud(table_right(:,5))<1/R_pre,1);
dist = sqrt((table_right(n,2)-table_right(n+1,2))^2+(table_right(n,3)-table_right(n+1,3))^2);
while dist < max_width && n > 1
    n = n-1;
    dist = sqrt((table_right(n,2)-table_right(n-1,2))^2+(table_right(n,3)-table_right(n-1,3))^2);
end
curvRight = table_right(n,5);

leftOrRight = abs(curvLeft-1/R_pre) < abs(curvRight-1/R_pre);
curvBest = leftOrRight*curvLeft + (~leftOrRight)*curvRight;
omegaBest = v_pre*curvBest;
RBest = 1/curvBest;

if sum(in_path_pre)<0.5
    RBest = R_pre;
end

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

% Plot all the obstacles with red x's
plot(x_obs_tot,y_obs_tot,'xr','MarkerSize',10,'LineWidth',2)
% And label them
for i = 1:num_obs  
    h = text(x_obs(i)+0.2,y_obs(i), sprintf('%d',i));
    set(h,'FontSize',17)
end

%{
for i = 1:length(x_obs)
    y_mid = 0:sign(R_left(i)+half_width-(-half_width))*0.1:2*R_left(i);
    y_left = -half_width:sign(R_left(i)+half_width-(-half_width))*0.1:2*R_left(i)+half_width;
    y_right = half_width:sign(R_left(i)+half_width-(-half_width))*0.1:2*R_left(i)-half_width;
    
    x_mid   = abs( sqrt(              2*R_left(i) - y_mid   ) .* sqrt(              y_mid   ));
    x_left  = abs( sqrt( half_width + 2*R_left(i) - y_left  ) .* sqrt( half_width + y_left  ));
    x_right = abs( sqrt( half_width - 2*R_left(i) + y_right ) .* sqrt( half_width - y_right ));
    
    plot(x_mid,y_mid,'--','Color',cols(mod(i,7)+1,:))
    plot(x_left,y_left,'Color',cols(mod(i,7)+1,:))
    plot(x_right,y_right,'Color',cols(mod(i,7)+1,:))
end
for i = 1:length(x_obs)
    y_mid = 0:sign(R_right(i)+half_width-(-half_width))*0.1:2*R_right(i);
    y_left = -half_width:sign(R_right(i)+half_width-(-half_width))*0.1:2*R_right(i)+half_width;
    y_right = half_width:sign(R_right(i)+half_width-(-half_width))*0.1:2*R_right(i)-half_width;
    
    x_mid   = abs( sqrt(              2*R_right(i) - y_mid   ) .* sqrt(              y_mid   ));
    x_left  = abs( sqrt( half_width + 2*R_right(i) - y_left  ) .* sqrt( half_width + y_left  ));
    x_right = abs( sqrt( half_width - 2*R_right(i) + y_right ) .* sqrt( half_width - y_right ));
    
    plot(x_mid,y_mid,'--','Color',cols(mod(i+num_obs,7)+1,:))
    plot(x_left,y_left,'Color',cols(mod(i+num_obs,7)+1,:))
    plot(x_right,y_right,'Color',cols(mod(i+num_obs,7)+1,:))
end
%}

% Plot the Best Path
y_mid = 0:sign(RBest+half_width-(-half_width))*0.1:2*RBest;
y_left = -half_width:sign(RBest+half_width-(-half_width))*0.1:2*RBest+half_width;
y_right = half_width:sign(RBest+half_width-(-half_width))*0.1:2*RBest-half_width;

x_mid   = abs( sqrt(              2*RBest - y_mid   ) .* sqrt(              y_mid   ));
x_left  = abs( sqrt( half_width + 2*RBest - y_left  ) .* sqrt( half_width + y_left  ));
x_right = abs( sqrt( half_width - 2*RBest + y_right ) .* sqrt( half_width - y_right ));

plot(x_mid,y_mid,'--r','LineWidth',2)
plot(x_left,y_left,'r','LineWidth',2)
plot(x_right,y_right,'r','LineWidth',2)
    
hold off
axis equal
xlim([0 xmax])
ylim([-ymax ymax])