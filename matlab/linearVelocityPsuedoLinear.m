function linearVelocity = linearVelocityPsuedoLinear(currentPose,goalPose)
maxAngularVelocity = .5;

angularVelocity = angularVelocityPsuedoLinear(currentPose,goalPose);
scalingFactor = (maxAngularVelocity - (abs(angularVelocity) -.1)) / maxAngularVelocity;
scalingFactor = min(1,max(scalingFactor,0));
% For now, give it a constant linear velocity.
linearVelocity = 0.5 * scalingFactor;
end