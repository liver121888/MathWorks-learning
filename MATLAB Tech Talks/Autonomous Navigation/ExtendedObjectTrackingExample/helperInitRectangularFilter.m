function filter = helperInitRectangularFilter(varargin)
% helperInitRectangularFilter A function to initialize the rectangular
% target PHD filter for the Extended Object Tracking example

% Copyright 2019 The MathWorks, Inc.

if nargin == 0
    % If called with no inputs, simply use the initctrectgmphd function to
    % create a PHD filter with no components.
    filter = initctrectgmphd;
    % Set process noise
    filter.ProcessNoise = diag([1 3]);
else
    % When called with detections input, add two components to the filter,
    % one for car and one for truck, More components can be added based on
    % prior knowledge of the scenario, example, pedestrian or motorcycle.
    % This is a "multi-model" type approach. Another approach can be to add
    % only 1 component with a higher covariance in the dimensions. The
    % later is computationally less demanding, but has a tendency to track
    % observable dimensions of the object. For example, if only the back is
    % visible, the measurement noise may cause the length of the object to
    % shrink.
    
    % Detections
    detections = varargin{1};
    
    % Create a GM-PHD filter with rectangular model
    filter = initctrectgmphd(detections);
    
    % Length width of a passenger car
    filter.States(6:7,1) = [4.7;1.8];
    
    % High certainty in dimensions
    lCov = 1e-4;
    wCov = 1e-4;
    lwCorr = 0.5;
    lwCov = sqrt(lCov*wCov)*lwCorr;
    filter.StateCovariances(6:7,6:7,1) = [lCov lwCov;lwCov wCov];
    
    % Add one more component by appending the filter with itself.
    append(filter,filter);
    
    % Set length and width to a truck dimensions
    filter.States(6:7,2) = [8.1;2.45];
    
    % Relative weights of each component
    filter.Weights = [0.7 0.3];
end

end


