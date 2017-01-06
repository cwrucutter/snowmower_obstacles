function angularVelocity = angularVelocityPsuedoLinear(currentPose,goalPose)

[~, dy, dtheta] = calcDxDyDthetaFromGoalInGoalFrame(currentPose,goalPose);
relativeGain = 2.0;
averageGain = 2.0;
returnToHeadingGain = (2.0*abs(averageGain))/(abs(relativeGain)+1);
returnToLineGain = abs(relativeGain)*returnToHeadingGain;
angularVelocity = -returnToHeadingGain*dtheta - returnToLineGain*dy;
angularVelocity =  limitMaxAngularVelocityAndAngularAcceleration(angularVelocity);
end

function angularVelocity = limitMaxAngularVelocityAndAngularAcceleration(angularVelocity)
% Limit maximum angular acceleration (TODO)
% Limit maximum angular velocity
maxAngularVelocity = 0.5;
if angularVelocity > maxAngularVelocity
    angularVelocity = maxAngularVelocity;
end
if angularVelocity < -maxAngularVelocity
    angularVelocity = -maxAngularVelocity;
end
end