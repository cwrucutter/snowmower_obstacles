% Initialize state
x = 0; y = 0; theta = 0;
X0 = [x; y; theta];
% Timing
tf = 5;

v = @(x) 1;
omega = @(x) 1;
[T,X] = simulateKinematics(X0, tf, v, omega);
% Plot
figure(2)
plot(X(:,1),X(:,2))
axis equal
%% Test calcDxDyDthetaFromGoalInGoalFrame
currentPose = [0, 0, 0];
goalPose = [-1, -1, -pi/4-.1];
[dx, dy, dtheta] = calcDxDyDthetaFromGoalInGoalFrame(currentPose, goalPose);
fprintf('dx = %g, dy = %g, dtheta = %g\n',dx,dy,dtheta);
figure(3)
% plot x-axis of currentPose frame
plot([currentPose(1) currentPose(1)+cos(currentPose(3))], [currentPose(2) currentPose(2)+sin(currentPose(3))])
hold on
% plot y-axis of currentPose frame
plot([currentPose(1) currentPose(1)-0.5*sin(currentPose(3))], [currentPose(2) currentPose(2)+0.5*cos(currentPose(3))])
% plot x-axis of goalPose frame
plot([goalPose(1) goalPose(1)+cos(goalPose(3))], [goalPose(2) goalPose(2)+sin(goalPose(3))])
% plot y-axis of goalPose frame
plot([goalPose(1) goalPose(1)-0.5*sin(goalPose(3))], [goalPose(2) goalPose(2)+0.5*cos(goalPose(3))])
axis equal
hold off