function [img,pose] = ExampleHelperSignFollowingROSProcessMsg(imgMsg,odomMsg)
% Extracts image and pose data from ROS messages

% Copyright 2019 The MathWorks, Inc.

% Extract image
img = rosReadImage(imgMsg);

% Extract 2D pose
ori = odomMsg.Pose.Pose.Orientation;
eul = quat2eul([ori.W, ori.X, ori.Y, ori.Z]);
pose = [odomMsg.Pose.Pose.Position.X; ...
        odomMsg.Pose.Pose.Position.Y; ... 
        eul(1)];

end


