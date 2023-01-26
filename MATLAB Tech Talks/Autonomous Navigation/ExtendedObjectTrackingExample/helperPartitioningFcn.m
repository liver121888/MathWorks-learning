function partitions = helperPartitioningFcn(detections, varargin)
% This is a helper function and may be removed or modified in a future
% release. This function calculates the partitions of a detection set for
% the "Extended Object Tracking of highway vehicles using radar and camera"
% example. 

% Copyright 2020 The MathWorks, Inc.

% This function removes the error due to range-rate measurements for the
% left and right mounted sensors. The radar sensor model outputs
% measurements which may not be separated by 3-5 resolutions in range-rate
% because of projection towards the sensor. For detections from left and
% right sensors, partition just using azimuth and range of the detections. 
if detections{1}.SensorIndex > 2
    for i = 1:numel(detections)
        detections{i}.Measurement(3) = 0;
    end
end

partitions = partitionDetections(detections,varargin{:});

end


