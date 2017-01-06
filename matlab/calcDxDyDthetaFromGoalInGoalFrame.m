function [dx, dy, dtheta] = calcDxDyDthetaFromGoalInGoalFrame(currentPose, goalPose)
% Inputs are 3x1 arrays, currentPose and goalPose. The indicies, in order,
% store the x, y, and heading values realtive to the global reference
% frame.
% Outputs are dx, dy, and dtheta. dx and dy are the perpendicular distances
% from the currentPose to the y-axis (along the x-axis) and to the x-axis 
% (along the y-axis) of the goalPose frame.
% dtheta is the angle of the goalPose heading relative to the currentPose
% heading, wrapped between [-pi, pi)
dx = calcDxFromGoalInGoalFrame(currentPose, goalPose);
dy = calcDyFromGoalInGoalFrame(currentPose, goalPose);
dtheta = calcDthetaFromGoalInGoalFrame(currentPose, goalPose);
end

function dx = calcDxFromGoalInGoalFrame(currentPose, goalPose)
% Calculate the perpendicular distance from the currentPose to the y-axis
% (along the x-axis) of the goalPose frame.
vectorFromCurrentPoseToGoalPose = [goalPose(1) - currentPose(1); ...
                                   goalPose(2) - currentPose(2)];
unitVectorAlongYAxisOfGoalFrame = [cos(goalPose(3)); sin(goalPose(3))];
dx = dot(vectorFromCurrentPoseToGoalPose, unitVectorAlongYAxisOfGoalFrame);
end

function dy = calcDyFromGoalInGoalFrame(currentPose, goalPose)
% Calculate the perpendicular distance from the currentPose to the x-axis
% (along the y-axis) of the goalPose frame.
vectorFromCurrentPoseToGoalPose = [goalPose(1) - currentPose(1); ...
                                   goalPose(2) - currentPose(2)];
unitVectorAlongXAxisOfGoalFrame = [-sin(goalPose(3)); cos(goalPose(3))];
dy = dot(vectorFromCurrentPoseToGoalPose, unitVectorAlongXAxisOfGoalFrame);
end

function dtheta = calcDthetaFromGoalInGoalFrame(currentPose, goalPose)
% Calculate the angle of the goal heading relative to the current heading.
% dtheta should lie in the semi-closed interval [-pi, pi)
dtheta = wrapAngleFromNegativepiTopi(goalPose(3)-currentPose(3));
end

function wrappedAngle = wrapAngleFromNegativepiTopi(angle)
% Wrap an angle such that it lies in the semi-closed interval [-pi, pi).
% If the angle is greater than or equal to pi, subtract pi until it isn't.
while angle >= pi
    angle = angle - pi;
end
% If the angle is less than pi, add pi until it isn't.
while angle < -pi
    angle = angle + pi;
end
wrappedAngle = angle;
end

