function reward = cartPoleRewardFunction(obs,action,nextObs)
% Compute reward value based on the next observation.

    if iscell(nextObs)
        nextObs = nextObs{1};
    end

    % Distance at which to fail the episode
    xThreshold = 2.4;

    % Reward each time step the cart-pole is balanced
    rewardForNotFalling = 1;

    % Penalty when the cart-pole fails to balance
    penaltyForFalling = -50;

    x = nextObs(1,:);
    distReward = 1 - abs(x)/xThreshold;

    isDone = cartPoleIsDoneFunction(obs,action,nextObs);

    reward = zeros(size(isDone));
    reward(logical(isDone)) = penaltyForFalling;
    reward(~logical(isDone)) = ...
        0.5 * rewardForNotFalling + 0.5 * distReward(~logical(isDone));
end
