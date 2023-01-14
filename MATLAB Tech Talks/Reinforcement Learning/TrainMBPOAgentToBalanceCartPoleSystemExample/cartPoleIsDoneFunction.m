function isDone = cartPoleIsDoneFunction(obs,action,nextObs)
% Compute termination signal based on next observation.

    if iscell(nextObs)
        nextObs = nextObs{1};
    end

    % Angle at which to fail the episode
    thetaThresholdRadians = 12 * pi/180;

    % Distance at which to fail the episode
    xThreshold = 2.4;

    x = nextObs(1,:);
    theta = nextObs(3,:);
    
    isDone = abs(x) > xThreshold | abs(theta) > thetaThresholdRadians;
end