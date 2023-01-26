function phd = helperInitGGIWFilter(varargin)
% helperInitGGIWFilter A function to initialize the GGIW-PHD filter for the
% Extended Object Tracking example

% Create a ggiwphd filter using 5 states and the constant turn-rate models.
phd = ggiwphd(zeros(5,1),eye(5),...
    'StateTransitionFcn',@constturn,...
    'StateTransitionJacobianFcn',@constturnjac,...
    'MeasurementFcn',@ctmeas,...
    'MeasurementJacobianFcn',@ctmeasjac,...
    'HasAdditiveMeasurementNoise',true,...
    'HasAdditiveProcessNoise',false,...
    'ProcessNoise',diag([1 1 3]),...
    'MaxNumComponents',1000,...
    'ExtentRotationFcn',@extentRotFcn,...
    'PositionIndex',[1 3]);

% If the function is called with no inputs i.e. the predictive portion of
% the birth density, no components are added to the mixture.
if nargin == 0
    % Nullify to return 0 components.
    nullify(phd);
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
    
    % Enable elevation measurements to create a 3-D filter using
    % initctggiwphd
    if detections{1}.SensorIndex < 7
        for i = 1:numel(detections)
            detections{i}.Measurement = [detections{i}.Measurement(1);0;detections{i}.Measurement(2:end)];
            detections{i}.MeasurementNoise = blkdiag(detections{i}.MeasurementNoise(1,1),0.4,detections{i}.MeasurementNoise(2:end,2:end));
            detections{i}.MeasurementParameters(1).HasElevation = true;
        end
    end
    phd3d = initctggiwphd(detections);
    
    % Set states of the 2-D filter using 3-D filter
    phd.States = phd3d.States(1:5);
    phd.StateCovariances = phd3d.StateCovariances(1:5,1:5);
    
    phd.DegreesOfFreedom = 1000;
    phd.ScaleMatrices = (1000-4)*diag([4.7/2 1.8/2].^2);
    
    % Add truck dimensions as second component
    append(phd,phd);
    phd.ScaleMatrices(:,:,2) = (1000-4)*diag([8.1/2 2.45/2].^2);
    phd.GammaForgettingFactors = [1.03 1.03];
    
    % Relative weights of the components. Can be treated as probability of
    % existence of a car vs a truck on road.
    phd.Weights = [0.7 0.3];
end
end

function R = extentRotFcn(x,dT)
    % Rotation of the extent during prediction.
    w = x(5);
    theta = w*dT;
    R = [cosd(theta) -sind(theta);sind(theta) cosd(theta)]; 
end