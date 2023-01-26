classdef helperMetricEvaluationDisplay < matlab.System
    % This is a helper class for the "Introduction to Tracking Metrics"
    % Example. It may be removed or modified in a future release
    
    % Copyright 2019 The MathWorks, Inc.
    
    % Plotters
    properties (Access = protected)
        TheaterPlot
        TrackPlotter
        TruthPlotter
        AssignmentPlotter
    end
    
    % Classification Plotter
    properties (Access = protected)
        RedundantTrackPlotter
        FalseTrackPlotter
        DivergentTrackPlotter
    end
    
    properties (Access = protected)
        EstablishedTruthPlotter
        BrokenTruthPlotter
    end
    
    properties (Access = protected)
        TrackErrorAxes
        TrackErrorPlotters;
        TrackErrorPlotterIDs;
        TruthErrorAxes
        TruthErrorPlotters;
        TruthErrorPlotterIDs;
        CurrentStep;
        pIsPublishing = false;
    end
    
    % GIF, Parent and Color Order
    properties
        Parent;
        ColorOrder = lines(7);
        RecordGIF = false;
        Frames
        DownsampleFactor = 1;
    end
    
    % Control what to plot
    properties
        PlotTrackClassification = true;
        PlotTruthClassification = true;
        PlotTrackErrors = false;
        PlotTruthErrors = false;
        PlotAssignments = true;
        ErrorToPlot = 'posRMS';
    end
    
    methods
        function obj = helperMetricEvaluationDisplay(varargin)
            setProperties(obj,nargin,varargin{:});
        end
        
        % Write animation if RecordGIF is true
        % writeAnimation(obj,fName) will write fName.GIF in the current
        % working folder
        function writeAnimation(obj,fName)
            if obj.RecordGIF
               frames = obj.Frames;
               imSize = size(frames{1}.cdata);
               im = zeros(imSize(1),imSize(2),1,floor(numel(frames)/obj.DownsampleFactor),'uint8');
               map = [];
               count = 1;
               for i = 1:obj.DownsampleFactor:numel(frames)
                   if isempty(map)
                       [im(:,:,1,count),map] = rgb2ind(frames{i}.cdata,256,'nodither');
                   else
                       im(:,:,1,count) = rgb2ind(frames{i}.cdata,map,'nodither');
                   end
                   count = count + 1;
               end
               imwrite(im,map,[fName,'.gif'],'DelayTime',0,'LoopCount',inf);
            end
        end
        
        % helperPlotStructArray plots a struct array with fields containing
        % the keyboard "Total"
        function helperPlotStructArray(~, st)
            fig = figure('Units','normalized','Position',[0.1 0.1 0.6 0.6]);
            ax = axes(fig);
            
            fNames = fieldnames(st);
            if isfield(st,'TotalNumTracks')
                titleStr = 'Assignment metrics for tracks';
            else
                titleStr = 'Assignment metrics for truths';
            end
            
            fNames = fNames(contains(fNames,{'Total'}));
            thisF = zeros(numel(st),numel(fNames));
            for i = 1:numel(fNames)
                thisF(:,i) = [st.(fNames{i})];
            end
            plot(ax, thisF,'LineWidth',2);
            maxY = max(thisF(:));
            minY = min(thisF(:));
            ylim(ax,[minY-1,maxY+1]);
            
            l = legend(fNames{:});
            l.Location = 'North outside';
            l.Orientation = 'horizontal';
            l.FontWeight = 'bold';
            xlabel(ax,'Time step');
            grid(ax,'on');  
            title(ax,titleStr);
        end
        
        % helperPlotTableArray(obj, tArray, error) plots the "error" from
        % cell array of tables.
        function helperPlotTableArray(~,tArray,errorToPlot)
            [maxIds,idx] = max(cellfun(@(x)size(x,1),tArray));
            data = nan(numel(tArray),maxIds);
            for i = 1:numel(tArray)
                thisData = tArray{i}.(errorToPlot);
                data(i,1:numel(thisData)) = thisData;
            end
            fig = figure('Units','normalized','Position',[0.1 0.1 0.6 0.6]);
            ax = axes(fig);
            grid(ax,'on');
            plot(ax,data,'LineWidth',2);
            maxY = max(data(:));
            minY = min(data(:));
            ylim(ax,[min(0.9*minY,-0.1),1.1*maxY]);
            
            varName = tArray{idx}.Properties.VariableNames{1};
            names = string([tArray{idx}.(varName)]);
            if strcmpi(varName,'TrackID')
                names = strcat("T",names);
                titleStr = 'Estimation error for each track';
            else
                names = strcat("P",names);
                titleStr = 'Estimation error for each truth';
            end
            title(ax,titleStr);
            l = legend(names);
            l.Location = 'North outside';
            l.Orientation = 'horizontal';
            l.FontWeight = 'bold';
            xlabel(ax,'Time step');
            switch errorToPlot
                case 'posRMS'
                    str = "RMS error in position";
                case 'velRMS'
                    str = "RMS error in velocity";
                case 'posNEES'
                    str = "Normalized error in position";
                case 'velNEES'
                    str = "Normalized error in velocity";
                otherwise
                    str = errorToPlot;
            end
            
            ylabel(ax,str);
            grid(ax,'on');
        end
    end

    methods (Access = protected)
        function setupImpl(obj,varargin)
            obj.Parent = figure('Units','normalized','Position',[0.1 0.1 0.8 0.8],...
                'Visible','on');
            if isPublishing(obj)
               obj.Parent.Visible = 'off'; 
               obj.pIsPublishing = true;
            end
            
            ax = axes(obj.Parent);
            obj.TheaterPlot = theaterPlot('Parent',ax,'XLimits',[-2 12],'YLimits',[-5 5]);
            co = obj.ColorOrder;
            if ~obj.PlotTrackClassification
                obj.TrackPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Tracks','MarkerEdgeColor',co(1,:),'MarkerFaceColor',co(1,:),'ConnectHistory','on');
            else
                obj.TrackPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Associated Tracks','MarkerFaceColor',co(1,:),'MarkerEdgeColor',co(1,:),'ConnectHistory','on');
                obj.RedundantTrackPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Redundant Tracks','MarkerFaceColor',co(2,:),'MarkerEdgeColor',co(2,:),'ConnectHistory','on');
                obj.FalseTrackPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','False Tracks','MarkerFaceColor',co(3,:),'MarkerEdgeColor',co(3,:),'ConnectHistory','on');
                obj.DivergentTrackPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Divergent Tracks','MarkerFaceColor',co(4,:),'MarkerEdgeColor',co(4,:),'ConnectHistory','on');
            end
            if ~obj.PlotTruthClassification
                obj.TruthPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Truths','MarkerFaceColor',co(2,:),'MarkerEdgeColor',co(2,:),'Marker','^','ConnectHistory','on');
            else
                obj.TruthPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Missed Target','MarkerFaceColor',co(5,:),'MarkerEdgeColor',co(5,:),'Marker','^','ConnectHistory','on');
                obj.EstablishedTruthPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Tracked Target','MarkerFaceColor',co(6,:),'MarkerEdgeColor',co(6,:),'Marker','^','ConnectHistory','on');
                obj.BrokenTruthPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Dropped Target','MarkerFaceColor',co(7,:),'MarkerEdgeColor',co(7,:),'Marker','^','ConnectHistory','on');
            end
            if obj.PlotAssignments
                hold (ax, 'on');
                legend(ax,'AutoUpdate','on');
                obj.AssignmentPlotter = plot3(ax, nan,nan,nan,'-.','Color','k','DisplayName','T2P Assignments');
            end
            if obj.PlotTrackErrors
                ax.Position = [0 0.5 1 0.4];
                title(ax,'Scenario');
                ax2 = axes(obj.Parent,'Units','normalized','Position',[0.1 0.1 0.35 0.35]);
                obj.TrackErrorAxes = ax2;
                hold(ax2,'on');
                title(ax2,'Track Error');
                xlabel(ax2,'Time step');
                ylabel(ax2,obj.ErrorToPlot);
                
                obj.TrackErrorPlotters = plot(ax2,nan,nan,'LineWidth',2);
                legend(ax2,"T1");
                obj.TrackErrorPlotterIDs = 1;
            end
            if obj.PlotTruthErrors
                ax3 = axes(obj.Parent,'Units','normalized','Position',[0.6 0.1 0.35 0.35]);
                obj.TruthErrorAxes = ax3;
                hold(ax3,'on');
                title(ax3,'Truth Error');
                xlabel(ax3,'Time step');
                ylabel(ax3,obj.ErrorToPlot);
                
                obj.TruthErrorPlotters = plot(ax3,nan,nan,'LineWidth',2);
                legend(ax3,"P1");
                obj.TruthErrorPlotterIDs = 1;
            end
            obj.CurrentStep = 0;
        end
        
        function stepImpl(obj, tracks, truths, varargin)
            if ~obj.pIsPublishing
                obj.CurrentStep = obj.CurrentStep + 1;
                plotTruths(obj, truths, varargin{:});
                plotTracks(obj, tracks, varargin{:});
                plotErrors(obj, tracks, truths, varargin{:});
                plotAssignments(obj, tracks, truths, varargin{:});
                if obj.RecordGIF
                    obj.Frames{end+1} = getframe(obj.Parent);
                end
            end
        end
        
        function plotErrors(obj, ~, ~, varargin)
            if obj.PlotTrackErrors
                trackErrors = varargin{5};
                errorValues = trackErrors.(obj.ErrorToPlot);
                errorIDs = trackErrors.TrackID;
                for i = 1:numel(errorIDs)
                    id = errorIDs(i);
                    plotTrackErrorValue(obj, errorValues(i), id);
                end
            end
            if obj.PlotTruthErrors
                truthErrors = varargin{6};
                errorValues = truthErrors.(obj.ErrorToPlot);
                errorIDs = truthErrors.TruthID;
                for i = 1:numel(errorIDs)
                    id = errorIDs(i);
                    plotTruthErrorValue(obj, errorValues(i), id);
                end
            end
        end
        
        function plotTrackErrorValue(obj, error, id)
            currentIDs = obj.TrackErrorPlotterIDs;
            plotters = obj.TrackErrorPlotters;
            plotter = plotters(currentIDs == id);
            if isempty(plotter)
                plotter = plot(obj.TrackErrorAxes,nan,nan,'LineWidth',2);
                obj.TrackErrorPlotters = [plotters;plotter];
                obj.TrackErrorPlotterIDs = [obj.TrackErrorPlotterIDs;id];
                l = legend(obj.TrackErrorAxes);
                l.String{end} = strcat("T",string(id));
            end
            plotter.XData = [plotter.XData obj.CurrentStep];
            plotter.YData = [plotter.YData error]; 
        end
        
        function plotTruthErrorValue(obj, error, id)
            currentIDs = obj.TruthErrorPlotterIDs;
            plotters = obj.TruthErrorPlotters;
            plotter = plotters(currentIDs == id);
            if isempty(plotter)
                plotter = plot(obj.TruthErrorAxes,nan,nan,'LineWidth',2);
                obj.TruthErrorPlotters = [plotters;plotter];
                obj.TruthErrorPlotterIDs = [obj.TruthErrorPlotterIDs;id];
                l = legend(obj.TruthErrorAxes);
                l.String{end} = strcat("P",string(id));
            end
            plotter.XData = [plotter.XData obj.CurrentStep];
            plotter.YData = [plotter.YData error]; 
        end
        
        function plotAssignments(obj, tracks, truths, varargin)
           if obj.PlotAssignments
               trackIDs = varargin{1};
               truthIDs = varargin{2};
           end
           
           inTrackIDs = vertcat(tracks.TrackID);
           inTruthIDs = vertcat(truths.PlatformID);
           
           plotLines = zeros(0,3);
           for i = 1:numel(trackIDs)
               if ~isnan(trackIDs(i)) && ~isnan(truthIDs(i))
                   thisTrack = tracks(inTrackIDs == trackIDs(i));
                   thisTruth = truths(inTruthIDs == truthIDs(i));
                   p1 = thisTrack.State(1:2:end);
                   p2 = thisTruth.Position;
                   plotLines(3*i-2,:) = p1;
                   plotLines(3*i-1,:) = p2;
                   plotLines(3*i,:) = nan;
               end
           end
           
           obj.AssignmentPlotter.XData = plotLines(:,1);
           obj.AssignmentPlotter.YData = plotLines(:,2);
           obj.AssignmentPlotter.ZData = plotLines(:,3);          
        end
        
        function plotTracks(obj, tracks, varargin)
            if obj.PlotTrackClassification
                trackTable = varargin{3};
                [associatedTracks, redundantTracks, falseTracks, divergentTracks] = classifyTracks(obj, trackTable, tracks);
                plotTracksOnPlotter(obj,associatedTracks,obj.TrackPlotter,varargin{:});
                plotTracksOnPlotter(obj,redundantTracks,obj.RedundantTrackPlotter,varargin{:});
                plotTracksOnPlotter(obj,falseTracks,obj.FalseTrackPlotter,varargin{:});
                plotTracksOnPlotter(obj,divergentTracks,obj.DivergentTrackPlotter,varargin{:});
            else
                plotTracksOnPlotter(obj, tracks, obj.TrackPlotter,varargin{:});
            end
        end
        
        function [associatedTracks, redundantTracks, falseTracks, divergentTracks] = classifyTracks(~, trackTable, tracks)
            metrics = trackTable;
            trkIDs = metrics.TrackID;
            isAssociated = ~isnan(metrics.AssignedTruthID);
            isRedundant = metrics.RedundancyStatus;
            isFalse = metrics.FalseTrackStatus;
            isDivergent = metrics.DivergenceStatus;
            
            inputTrackIDs = vertcat(tracks.TrackID);
            
            associatedTracks = tracks(ismember(inputTrackIDs,trkIDs(isAssociated)));
            redundantTracks = tracks(ismember(inputTrackIDs,trkIDs(isRedundant)));
            falseTracks = tracks(ismember(inputTrackIDs,trkIDs(isFalse)));
            divergentTracks = tracks(ismember(inputTrackIDs,trkIDs(isDivergent)));
        end
        
        function plotTracksOnPlotter(~, tracks, plotter,varargin)
            posSelector = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];
            
            pos = getTrackPositions(tracks, posSelector);
            if ~isempty(pos)
                trkLabels = strcat("T",string([tracks.TrackID]));
                plotter.plotTrack(pos, trkLabels, vertcat(tracks.TrackID));
            else
                plotter.plotTrack([nan nan nan],{'1'},1);
            end
        end
                
        function plotTruths(obj, truths, varargin)
            if obj.PlotTruthClassification
                truthTable = varargin{4};
                [unestablishedTruth, establishedTruth, brokenTruth] = classifyTruths(obj, truthTable, truths);
                plotTruthsOnPlotter(obj, unestablishedTruth, obj.TruthPlotter,varargin{:});
                plotTruthsOnPlotter(obj, establishedTruth, obj.EstablishedTruthPlotter,varargin{:});
                plotTruthsOnPlotter(obj, brokenTruth, obj.BrokenTruthPlotter,varargin{:});
            else
                plotTruthsOnPlotter(obj, truths, obj.TruthPlotter, varargin{:});
            end
        end
        
        function [unestablishedTruth, establishedTruth, brokenTruth] = classifyTruths(~, truthTable, truths)
            metrics = truthTable;
            truthIDs = metrics.TruthID;
            isEstablished = metrics.EstablishmentStatus;
            isUnestablished = ~isEstablished;
            isBroken = metrics.BreakStatus;
            
            inputTruthIDs = vertcat(truths.PlatformID);
            
            establishedTruth = truths(ismember(inputTruthIDs,truthIDs(isEstablished)));
            unestablishedTruth = truths(ismember(inputTruthIDs,truthIDs(isUnestablished)));
            brokenTruth = truths(ismember(inputTruthIDs, truthIDs(isBroken)));
        end
        
        function plotTruthsOnPlotter(~, truths, plotter,varargin)
            pos = vertcat(truths.Position);
            vel = vertcat(truths.Velocity);
            if ~isempty(pos)
                trthLabels = strcat("P",string([truths.PlatformID]));
                plotter.plotTrack(pos, vel, cellstr(trthLabels), vertcat(truths.PlatformID));
            else
                plotter.plotTrack([nan nan nan],{'1'},1);
            end
        end
        
        function tf = isPublishing(~)
           % A check if the examples is being "published"
           s = dbstack;
           tf = numel(s) > 5;
        end
    end
end
