function [scenario, egoCar, sensors] = helperCreateScenario
% Scenario generation comprises generating a road network, defining
% vehicles that move on the roads, and moving the vehicles.
% 
% In this example, you test the ability of the sensor fusion to track a
% vehicle that is passing on the left of the ego vehicle. The scenario
% simulates a highway setting, and additional vehicles are in front of and
% behind the ego vehicle.

% Copyright 2018-2022 The Mathworks, Inc.

% Define an empty scenario.
scenario = drivingScenario;
scenario.SampleTime = 0.1;

%% 
% Add a stretch of 500 meters of typical highway road with two lanes. The 
% road is defined using a set of points, where each point defines the center of 
% the road in 3-D space. 
roadCenters = [0 0; 50 0; 100 0; 250 20; 400 35];
road(scenario, roadCenters, 'lanes',lanespec(3));

%% 
% Create the ego vehicle and three cars around it: one that overtakes the
% ego vehicle and passes it on the left, one that drives right in front of
% the ego vehicle and one that drives right behind the ego vehicle. All the
% cars follow the trajectory defined by the road waypoints by using the
% |trajectory| driving policy. The passing car will start on the right
% lane, move to the left lane to pass, and return to the right lane.

% Create the ego vehicle that travels at 25 m/s along the road.  Place the
% vehicle on the right lane by subtracting off half a lane width (1.8 m)
% from the centerline of the road.
egoCar = vehicle(scenario, 'ClassID', 1);
trajectory(egoCar, roadCenters(2:end,:) - [0 0], 25); % On right lane

% Add a truck in front of the ego vehicle
truck = vehicle(scenario, 'ClassID', 1, 'Length', 8.1, 'Width', 2.45, 'Height',4);
trajectory(truck, [86 0;roadCenters(3:end,:)] + [0 -3.6], 25); % On center lane

% % Add a passing car that travels at 35 m/s along the road
passingCar = vehicle(scenario, 'ClassID', 1, 'Length', 4.7, 'Width', 1.8, 'Height', 1.4);
waypoints = [0 0; 50 1.8; 100 1.8; 250 21.8; 400 36.8] + [0 1.8];
trajectory(passingCar, waypoints, 35);

% % Add a car behind the ego vehicle
chaseCar = vehicle(scenario, 'ClassID', 1, 'Length', 4.7, 'Width', 1.8, 'Height', 1.4);
trajectory(chaseCar, [30 0; roadCenters(2:end,:)] - [0 0], 25); % On right lane

% A car in front of ego
followCar = vehicle(scenario, 'ClassID', 1, 'Length', 4.7, 'Width', 1.8, 'Height', 1.4);
trajectory(followCar, [75 0; roadCenters(3:end,:)] - [0 0], 25); % On right lane

%% 
% Define Radar and Vision Sensors
% In this example, you simulate an ego vehicle that has 6 radar sensors and
% 2 vision sensors covering the 360 degrees field of view. The sensors have
% some overlap and some coverage gap. The ego vehicle is equipped with a
% long-range radar sensor and a vision sensor on both the front and the
% back of the vehicle. Each side of the vehicle has two short-range radar
% sensors, each covering 90 degrees. One sensor on each side covers from
% the middle of the vehicle to the back. The other sensor on each side
% covers from the middle of the vehicle forward. The figure in the next
% section shows the coverage.

sensors = cell(8,1);

% Front-facing long-range radar sensor at the center of the front bumper of the car.
sensors{1} = drivingRadarDataGenerator('SensorIndex', 1,...
    'RangeLimits',[0 174], ...
    'MountingLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0 0.2],...
    'FieldOfView', [20, 5]...
    );

% Rear-facing long-range radar sensor at the center of the rear bumper of the car.
sensors{2} = drivingRadarDataGenerator('SensorIndex', 2, ...
    'RangeLimits',[0 174],...
    'MountingLocation',[-egoCar.RearOverhang, 0, 0.2],...
    'MountingAngles',[180 0 0],...
    'FieldOfView',[20 5]...
    );

% Rear-left-facing short-range radar sensor at the left rear wheel well of the car.
sensors{3} = drivingRadarDataGenerator('SensorIndex', 3, ...
    'RangeLimits',[0 30],...
    'MountingLocation',[0, egoCar.Width/2 0.2],...
    'MountingAngles',[120 0 0],...
    'FieldOfView',[90 5],...
    'ReferenceRange',50,...
    'AzimuthResolution',5,...
    'RangeResolution',1.25...
    );

% Rear-right-facing short-range radar sensor at the right rear wheel well of the car.
sensors{4} = drivingRadarDataGenerator('SensorIndex', 4, ...
    'RangeLimits',[0 30],...
    'MountingLocation',[0, -egoCar.Width/2 0.2],...
    'MountingAngles',[-120 0 0],...
    'FieldOfView',[90 5],...
    'ReferenceRange',50,...
    'AzimuthResolution',5,...
    'RangeResolution',1.25...
    );

% Front-left-facing short-range radar sensor at the left front wheel well of the car.
sensors{5} = drivingRadarDataGenerator('SensorIndex', 5, ...
    'RangeLimits',[0 30],...
    'MountingLocation',[egoCar.Wheelbase, egoCar.Width/2 0.2],...
    'MountingAngles',[60 0 0],...
    'FieldOfView',[90 5],...
    'ReferenceRange',50,...
    'AzimuthResolution',5,...
    'RangeResolution',1.25);

% Front-right-facing short-range radar sensor at the right front wheel well of the car.
sensors{6} = drivingRadarDataGenerator('SensorIndex', 6, ...
    'RangeLimits',[0 30],...
    'MountingLocation',[egoCar.Wheelbase, -egoCar.Width/2 0.2],...
    'MountingAngles',[-60 0 0],...
    'FieldOfView',[90 5],...
    'ReferenceRange',50,...
    'AzimuthResolution',5,...
    'RangeResolution',1.25,...
    'HasOcclusion',true);

% Front-facing camera located at front windshield.
sensors{7} = visionDetectionGenerator('SensorIndex', 7, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], 'Height', 1.1,'DetectionProbability',0.99,...
    'MaxAllowedOcclusion',0.5);

% Rear-facing camera located at rear windshield.
sensors{8} = visionDetectionGenerator('SensorIndex', 8, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.2*egoCar.Wheelbase 0], 'Height', 1.1, 'Yaw', 180,'DetectionProbability',0.99,...
    'MaxAllowedOcclusion',0.5);

for i = 1:6
    sensors{i}.TargetReportFormat = 'Detections';
    sensors{i}.DetectionCoordinates = 'Sensor Spherical';
    sensors{i}.HasFalseAlarms = true;
    sensors{i}.FalseAlarmRate = 1e-6;
    sensors{i}.HasNoise = true;
    sensors{i}.HasRangeRate = true;
    sensors{i}.Profiles = actorProfiles(scenario);
    sensors{i}.HasOcclusion = false;
end

for i = 7:8
    sensors{i}.DetectionCoordinates = 'Sensor Cartesian';
    sensors{i}.HasNoise = true;
    sensors{i}.FalsePositivesPerImage = 0.1;
    sensors{i}.ActorProfiles = actorProfiles(scenario);
end
end