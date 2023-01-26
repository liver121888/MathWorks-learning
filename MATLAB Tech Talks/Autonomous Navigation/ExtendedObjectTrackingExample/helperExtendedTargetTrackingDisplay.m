classdef helperExtendedTargetTrackingDisplay < matlab.System
    % This is a helper class for Extended Object Tracking example to
    % display tracks, detections and ground truth. It may be removed or
    % modified in a future release.
    
    % Copyright 2019 The MathWorks, Inc.
    
    % Public properties
    properties
        Figure
        BirdsEyePlots
        % ActorID to follow in the center panel
        FollowActorID = 3;
        % Plot clustered radar detections?
        PlotClusteredDetection = true;
    end
    
    methods
        function obj = helperExtendedTargetTrackingDisplay(varargin)
            setProperties(obj,nargin,varargin{:});
            % Make a figure
            hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Extended Object Tracking Example');
            set(hFigure,'Visible','off')
            movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top
            obj.Figure = hFigure;
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, egoVehicle, sensors)
            obj.BirdsEyePlots = createDemoDisplay(obj,egoVehicle,sensors);
        end
        
        function stepImpl(obj, egoVehicle, ~,  detections,tracks, varargin)
            bep = obj.BirdsEyePlots;
            % Update plots
            helperUpdateDisplayExtended(bep,egoVehicle,detections,tracks, varargin{:});
            
            % Follow actor ID provided in actor ID
            if ~isempty(obj.FollowActorID)
                scene = egoVehicle.Scenario;
                pos = targetOutlines(egoVehicle);
                actorPos = pos(ismember([scene.Actors.ActorID],obj.FollowActorID),:);
                minX = min(actorPos(:,1));
                maxX = max(actorPos(:,1));
                minY = min(actorPos(:,2));
                maxY = max(actorPos(:,2));
                bep{2}.XLimits = [minX-25 maxX+25];
                bep{2}.YLimits = [minY-15 maxY+15];
            end
        end
        
        function BEPS = createDemoDisplay(obj, egoCar, sensors)
            hFigure = obj.Figure;
            
            % Add a car plot that follows the ego vehicle from behind
            hCarViewPanel = uipanel(hFigure, 'Position', [0.005 0.75 0.25 0.25], 'Title', 'Chase Camera View');
            hCarPlot = axes(hCarViewPanel);
            chasePlot(egoCar, 'Parent', hCarPlot);
            
            % Create panels with bird's-eye plots
            BEP = createBEPPanel(obj, hFigure, [0.005 0 0.25 0.75], 60, sensors, 'Bird''s-Eye Plot', false);
            PassingBEP = createBEPPanel(obj, hFigure, [0.255 0 0.37 1], 40, {}, 'Passing Vehicle', true);
            CenterBEP = createBEPPanel(obj, hFigure, [0.63 0 0.37 1], 40, {}, 'Ego Vehicle', true);
            BEPS = {BEP,PassingBEP,CenterBEP};
            if isempty(snapnow('get'))
                set(hFigure,'Visible','on')
            end
        end
    end
end


function BEP = createBEPPanel(obj, hFigure, position, frontBackLim, sensors, title, isLegend)
    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', position, 'Title', title);
    
    % Create bird's-eye plot for the ego car and sensor coverage
    hBEVPlot = axes(hBEVPanel);
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);
    
    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-frontBackLim frontBackLim]*0.667);
    legend(hBEVPlot, 'off')
    
    % Create a vision detection plotter
    detectionPlotter(BEP, 'DisplayName','vision','MarkerEdgeColor','blue', 'MarkerFaceColor','blue','Marker','^','MarkerSize',6);
    
    % Create a radar detection plotter
    detectionPlotter(BEP, 'DisplayName','radar', 'Marker','o','MarkerEdgeColor','red','MarkerFaceColor','red','MarkerSize',6);
    
    if obj.PlotClusteredDetection
        % Create a clustered detection plotter
        trackPlotter(BEP, 'DisplayName','radar cluster', 'Marker','*','MarkerEdgeColor','red','MarkerFaceColor','red','MarkerSize',6);
    end
    
    % Create a lane marking plotter
    laneMarkingPlotter(BEP, 'DisplayName','lane');
    
    % Create a track plotter
    trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);
  
    % Add an outline plotter for the track outlines of Rectangular targets
    p = trackOutlinePlotter(BEP, 'Tag', 'Track Outlines','FaceAlpha',0);
    p.LineStyle = ':';
    
    % Add an outline plotter for ground truth
    outlinePlotter(BEP, 'Tag', 'Ground truth');
    
    % Add a plotter for extent for plotting track outlines of Elliptical
    % targets
    trackPlotter(BEP, 'Tag', 'Track Extent');
    s = findall(BEP.Parent,'Tag','bepTracksCovariances');
    s.LineStyle = ':';
    s.LineWidth = 2;
    
    if ~isempty(sensors)
        % Plot the coverage areas for radars
        for i = 1:6
            cap = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
            plotCoverageArea(cap, sensors{i}.MountingLocation(1:2),...
                sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(1), sensors{i}.FieldOfView(1));
        end

        % Plot the coverage areas for vision sensors
        for i = 7:8
            cap = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
            plotCoverageArea(cap, sensors{i}.SensorLocation,...
                sensors{i}.MaxRange, sensors{i}.Yaw, 45);
        end
    end
    
    if ~isLegend
        
    else
        legend('Orientation','horizontal','NumColumns',3)
        legend('Location', 'NorthOutside')
    end
end

function helperUpdateDisplayExtended(BEPS, egoCar, detections, confirmedTracks, varargin)
%%% 
% helperUpdateDisplayExtended  Helper to update the display with extended tracks
% 
% This function updates the bird's-eye plot with road boundaries,
% detections, and extended tracks.
    for b = 1:numel(BEPS)
        BEP = BEPS{b};
        helperUpdateDisplayNonTracks(BEP, egoCar, detections, varargin{:});
        trackIDs = [confirmedTracks.TrackID];
        if isfield(confirmedTracks,'SourceIndex') % PHD
            if isfield(confirmedTracks,'Extent') % GGIW-PHD
                [trackPos, trackExtent] = trackOutlinesGGIW(confirmedTracks);
                plotTrack(findPlotter(BEP, 'Tag','Track Extent'), trackPos, trackExtent, string(trackIDs));
            else
                [tracksPos, yaw, length, width] = tracksOutlines(confirmedTracks); % GM-PHD
                plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, string(trackIDs));
                plotOutline(findPlotter(BEP,'Tag','Track Outlines'), tracksPos, yaw, length, width,...
                    'Color', ones(numel(confirmedTracks), 3) * 0.1 ...
                    );
            end
        elseif isa(confirmedTracks,'objectTrack') % multiObjectTracker
            [trackPos, trackPosCov] = getTrackPositions(confirmedTracks,[1 0 0 0 0;0 0 1 0 0]);
            plotTrack(findPlotter(BEP,'DisplayName','track'), trackPos, trackPosCov, string(trackIDs));
        end
    end
end


function [pos,extent] = trackOutlinesGGIW(tracks)
    pos = zeros(numel(tracks),2);
    extent = zeros(2,2,numel(tracks));
    for i = 1:numel(tracks)
        pos(i,:) = tracks(i).State([1 3]);
        extent(:,:,i) = tracks(i).Extent;
    end
end

function [position, yaw, length, width] = tracksOutlines(tracks)
% tracksOutlines  Returns the track outlines for display

position = zeros(numel(tracks), 2);
yaw = zeros(numel(tracks), 1);
length = zeros(numel(tracks), 1);
width = zeros(numel(tracks), 1);

for i = 1:numel(tracks)
    position(i, :) = tracks(i).State(1:2)';
    yaw(i) = tracks(i).State(4);
    length(i, 1) = tracks(i).State(6);
    width(i, 1) = tracks(i).State(7);
end
end

function helperUpdateDisplayNonTracks(BEP,egoCar,detections, detectionCluster)
%helperUpdateDisplayNonTracks  Helper to update display of all non-track plotters

% Update road boundaries and their display
[lmv, lmf] = laneMarkingVertices(egoCar);
plotLaneMarking(findPlotter(BEP,'DisplayName','lane'),lmv,lmf);

phi = egoCar.Yaw;
rot = [cosd(phi), sind(phi); -sind(phi), cosd(phi)];
% update ground truth data
[position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);

% Prepare and update detections display
sIdx = cellfun(@(x)x.SensorIndex,detections);
uqSIdx = unique(sIdx);
posEgo = zeros(3,numel(detections));
isRadar = sIdx <= 6;
for i = 1:numel(uqSIdx)
    thisIdx = sIdx == uqSIdx(i);
    posEgo(:,thisIdx) = calculatePositionInEgoFrame(detections(thisIdx));
end

plotDetection(findPlotter(BEP,'DisplayName','vision'), posEgo(1:2,~isRadar)');
plotDetection(findPlotter(BEP,'DisplayName','radar'), posEgo(1:2,isRadar)');

if nargin > 3
    sIdx = cellfun(@(x)x.SensorIndex,detectionCluster);
    uqSIdx = unique(sIdx);
    posEgo = zeros(3,numel(detectionCluster));
    posCovEgo = zeros(3,3,numel(detectionCluster));
    isRadar = sIdx <= 6;
    for i = 1:numel(uqSIdx)
        thisIdx = sIdx == uqSIdx(i);
        [posEgo(:,thisIdx),~,posCovEgo(:,:,thisIdx)] = calculatePositionInEgoFrame(detectionCluster(thisIdx));
    end
    plotTrack(findPlotter(BEP,'DisplayName','radar cluster'), posEgo(1:2,isRadar)', posCovEgo(1:2,1:2,isRadar));
end

end

function [posEgo, velEgo, posCovEgo] = calculatePositionInEgoFrame(detections)

% Calculate Cartesian positions for all detections in the "sensor"
% coordinate frame
allDets = [detections{:}];
meas = horzcat(allDets.Measurement);

if strcmpi(allDets(1).MeasurementParameters(1).Frame,'Spherical')
    az = meas(1,:);
    r = meas(2,:);
    el = zeros(1,numel(az));
    [x, y, z] = sph2cart(deg2rad(az),deg2rad(el),r);
    posSensor = [x;y;z];
    rr = meas(3,:);
    rVec = posSensor./sqrt(dot(posSensor,posSensor,1));
    velSensor = rr.*rVec;
else
    posSensor = meas;
    velSensor = zeros(3,size(meas,2));
end

% Transform parameters
sensorToEgo = detections{1}.MeasurementParameters(1);
R = sensorToEgo.Orientation;
T = sensorToEgo.OriginPosition;
if isfield(sensorToEgo,'OriginVelocity')
    Tdot = sensorToEgo.OriginVelocity;
else
    Tdot = zeros(3,1);
end

if isfield(sensorToEgo,'IsParentToChild') && sensorToEgo.IsParentToChild
    R = R';
end

% Position, velocity in ego frame
posEgo = T + R*posSensor;
velEgo = Tdot + R*velSensor; % Assume Rdot = 0;

if nargout > 2
    assert(~strcmpi(allDets(1).MeasurementParameters(1).Frame,'Spherical'),'Only cartesian measurements');
    measCov = cat(3,allDets.MeasurementNoise);
    posCovEgo = zeros(3,3,numel(allDets));
    for i = 1:numel(allDets)
        posCovEgo(:,:,i) = R*measCov(:,:,i)*R';
    end
end
end

