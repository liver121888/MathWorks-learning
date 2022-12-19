function DeploySignFollowingRobotROS
%#codegen

imgSub = rossubscriber("/camera/rgb/image_raw","sensor_msgs/Image","DataFormat","struct");

odomSub = rossubscriber("/odom","nav_msgs/Odometry","DataFormat","struct");

[velPub, velMsg] = rospublisher("/cmd_vel", "geometry_msgs/Twist","DataFormat","struct");

colorThresholds = [100 255 0 55 0 50; ... % Red
                   0 50 50 255 0 50; ...  % Green
                   0 40 0 55 50 255]';    % Blue

% Load Controller
controller = ExampleHelperSignFollowingControllerChart;

% Control the visualization of the mask. Keep this false for code
% generation.
doVisualization = false;

r = rosrate(10);
receive(imgSub); % Wait to receive an image message before starting the loop
receive(odomSub);

% Main controller loop
while(~controller.done)
    % Get latest sensor messages and process them
    imgMsg = imgSub.LatestMessage;
    odomMsg = odomSub.LatestMessage;
    [img,pose] = ExampleHelperSignFollowingROSProcessMsg(imgMsg, odomMsg);
    
    % Run vision and control functions
    [mask,blobSize,blobX] = ExampleHelperSignFollowingProcessImg(img, colorThresholds);
    step(controller,'blobSize',blobSize,'blobX',blobX,'pose',pose);
    v = controller.v;
    w = controller.w;
    
    % Publish velocity commands
    velMsg.Linear.X = v;
    velMsg.Angular.Z = w;
    send(velPub,velMsg);
    
    % Optionally visualize
    % NOTE: Visualizing data will slow down the execution loop.
    % If you have Computer Vision Toolbox, we recommend using
    % vision.DeployableVideoPlayer instead of imshow.
    if doVisualization
        imshow(mask);
        title(['Linear Vel: ' num2str(v) ' Angular Vel: ' num2str(w)]);
        drawnow('limitrate');
    end
    
    % Pace the execution loop.
    waitfor(r);
end

end