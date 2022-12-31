function [f,plotters] = createT2TDisplay(scenario, sensors, attachedVehicle)
% Create bird's eye plot
f = figure('Visible','off'); pause(0.1)
set(f,'Position',[433 425 1362 607]);
p1 = uipanel(f,'Title', 'Vehicle 1', 'Position', [0.01 0.01 0.48 0.98]);
p2 = uipanel(f,'Title', 'Vehicle 2', 'Position', [0.51 0.01 0.48 0.98]);
a = axes(p1);
xlims = [50 220];
bep{1} = birdsEyePlot('Parent', a, 'XLim', xlims, 'YLim', [-50 50]);
a = axes(p2);
bep{2} = birdsEyePlot('Parent', a, 'XLim', xlims, 'YLim', [-50 50]);
set(p1.Children(1),'Position',[0.35 0.95 0.3 0.0345],'Orientation','horizontal','NumColumnsMode','manual','NumColumns',5);
set(p2.Children(1),'Position',[0.35 0.95 0.3 0.0345],'Orientation','horizontal','NumColumnsMode','manual','NumColumns',5);
for i = 1:numel(sensors)
    shift = scenario.Actors(attachedVehicle(i)).Position(1:2);
    name = "";%Vehicle " + string(attachedVehicle(i));
    if isa(sensors{i},'radarDetectionGenerator')
        plotters.SensorPlotter(i) = coverageAreaPlotter(bep{attachedVehicle(i)}, 'DisplayName',...
            name + " radar", 'FaceColor', 'r');
        plotCoverageArea(plotters.SensorPlotter(i), sensors{i}.SensorLocation + shift,...
            sensors{i}.MaxRange, sensors{i}.Yaw,...
            sensors{i}.FieldOfView(1));
    else
        plotters.SensorPlotter(i) = coverageAreaPlotter(bep{attachedVehicle(i)}, 'DisplayName',...
            name + " vision", 'FaceColor', 'b');
        plotCoverageArea(plotters.SensorPlotter(i), sensors{i}.SensorLocation + shift,...
            sensors{i}.MaxRange, sensors{i}.Yaw,...
            sensors{i}.FieldOfView(1));
    end
end
plotters.veh1DetPlotter = detectionPlotter(bep{1}, 'DisplayName', 'Detections',...
    'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black');
plotters.veh2DetPlotter = detectionPlotter(bep{2}, 'DisplayName', 'Detections',...
    'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black');
plotters.veh1TrkPlotter = trackPlotter(bep{1}, 'DisplayName', 'Local Tracks',...
    'MarkerEdgeColor', 'black');
plotters.veh2TrkPlotter = trackPlotter(bep{2}, 'DisplayName', 'Local Tracks',...
    'MarkerEdgeColor', 'black');
plotters.veh1FusePlotter = trackPlotter(bep{1}, 'DisplayName', 'Fuser Tracks',...
    'MarkerEdgeColor', 'black', 'Marker', 'd');
plotters.veh2FusePlotter = trackPlotter(bep{2}, 'DisplayName', 'Fuser Tracks',...
    'MarkerEdgeColor', 'black', 'Marker', 'd');
plotters.ol1Plotter = outlinePlotter(bep{1});
plotters.lb1Plotter = laneBoundaryPlotter(bep{1});

plotters.ol2Plotter = outlinePlotter(bep{2});
plotters.lb2Plotter = laneBoundaryPlotter(bep{2});

rb = roadBoundaries(scenario);
[position, yaw, length, width, originOffset, color] = targetOutlines(scenario.Actors(1));
position = position + scenario.Actors(1).Position(1:2);

% update the bird's-eye plotters with the road and actors
plotLaneBoundary(plotters.lb1Plotter, rb);
plotOutline(plotters.ol1Plotter, position, yaw, length, width, ...
    'OriginOffset', originOffset, 'Color', color);
plotLaneBoundary(plotters.lb2Plotter, rb);
plotOutline(plotters.ol2Plotter, position, yaw, length, width, ...
    'OriginOffset', originOffset, 'Color', color);
end