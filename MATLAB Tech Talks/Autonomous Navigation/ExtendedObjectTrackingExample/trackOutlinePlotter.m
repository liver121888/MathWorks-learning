classdef trackOutlinePlotter < driving.birdsEyePlot.BirdsEyePlotter
%
%   See:  <a href="matlab:help birdsEyePlot/outlinePlotter">birdsEyePlot/outlinePlotter</a> 

%   Copyright 2017 The MathWorks, Inc.

    properties (Dependent)
        FaceAlpha
        LineStyle
    end

    properties (Dependent, Hidden)
        HistoryDepth
    end

    properties (Access = private)
        EgoPatch = matlab.graphics.primitive.Patch.empty;
        ActorPatches = matlab.graphics.primitive.Patch.empty;
        pHistoryDepth = 0
        pHistoryIndex = 1
        pPlotMethodParser
    end
    
    methods
        function obj = trackOutlinePlotter(bep, varargin)
            obj@driving.birdsEyePlot.BirdsEyePlotter(bep, varargin{:});
            initPlotMethodParser(obj);
        end

        function value = get.FaceAlpha(obj)
            hPatch = obj.EgoPatch;
            value = hPatch.FaceAlpha;
        end
        
        function value = get.LineStyle(obj)
            hPatch = obj.EgoPatch;
            value = hPatch.LineStyle;
        end

        function set.FaceAlpha(obj,value)
            hPatch = obj.EgoPatch;
            if ishghandle(hPatch)
                hPatch.FaceAlpha = value;
                ap = obj.ActorPatches(:);
                set(ap(ishghandle(ap)),'FaceAlpha',value);
            end
            drawnow limitrate;
        end
        
        function set.LineStyle(obj,value)
            hPatch = obj.EgoPatch;
            if ishghandle(hPatch)
                hPatch.LineStyle = value;
                ap = obj.ActorPatches(:);
                set(ap(ishghandle(ap)),'LineStyle',value);
            end
            drawnow limitrate;
        end
        
        function value = get.HistoryDepth(obj)
            value = obj.pHistoryDepth;
        end
        
        function set.HistoryDepth(obj, value)
            validateattributes(value,{'numeric'},{'scalar','integer','nonnegative','<=',100});
            obj.pHistoryDepth = value;
            clearData(obj);
        end
        
        function plotOutline(obj, varargin)
            [position, rYaw, rLength, rWidth, originOffset, color] = parsePlotMethod(obj, varargin);
            validateOutlines(obj, position, rYaw, rLength, rWidth, originOffset, color);
            plotActorPatches(obj, position, rYaw, rLength, rWidth, originOffset, color);
            drawnow limitrate;
        end
        
        function clearData(obj)
            delete(obj.ActorPatches(ishghandle(obj.ActorPatches(:))));
            obj.ActorPatches = matlab.graphics.primitive.Patch.empty;
            obj.pHistoryIndex = 1;
            drawnow limitrate;
        end
    end
    
    methods (Access = protected)
        function initParser(obj, p)
            initParser@driving.birdsEyePlot.BirdsEyePlotter(obj, p)
            addParameter(p,'HistoryDepth',0);
            addParameter(p,'FaceAlpha',0.75);
            addParameter(p,'LineStyle',0.75);
        end
        
        function initPlotMethodParser(obj)
            p = inputParser;
            p.addRequired('position');
            p.addRequired('yaw');
            p.addRequired('length');
            p.addRequired('width');
            p.addParameter('OriginOffset',[]);
            p.addParameter('Color',[]);
            p.addParameter('DisplayName',[]);
            obj.pPlotMethodParser = p;
        end
        
        function [position, rYaw, rLength, rWidth, originOffset, color] = parsePlotMethod(obj, arglist)
            p = obj.pPlotMethodParser;
            p.parse(arglist{:});
            pr = p.Results;
            pu = p.Unmatched;
            position = pr.position;
            if isempty(position)
                position = zeros(0,2);
            end
            rLength = pr.length;
            rWidth = pr.width;
            rYaw = pr.yaw;
            if isfield(pu,'OriginOffset') || isempty(pr.OriginOffset)
                originOffset = zeros(size(position,1),2);
            else
                originOffset = pr.OriginOffset;
            end
            if isfield(pu,'Color') || isempty(pr.Color)
                color = lines(size(position,1));
            else
                color = pr.Color;
            end
            if ~isfield(pu,'DisplayName') && ~isempty(pr.DisplayName)
                error(message('driving:birdsEyePlot:DisplayNameUnsupportedForOutlinePlotter'));
            end
        end
        
        function value = getPrimaryObject(obj)
            value = obj.EgoPatch;
        end
        
        function createGraphics(obj, hAxes, pr)
            if ~isempty(pr.DisplayName)
                error(message('driving:birdsEyePlot:DisplayNameUnsupportedForOutlinePlotter'));
            end
            hPatch = patch(hAxes, 0, 0, [0 0 0], ...
                         'HandleVisibility','off', ...
                         'Tag', 'bepEgoContainer');
            set(hPatch,'XData',[],'YData',[]);
            obj.EgoPatch = hPatch;
            obj.HistoryDepth = pr.HistoryDepth;
            obj.FaceAlpha = pr.FaceAlpha;
        end
        
        function deleteGraphics(obj)
            delete(obj.EgoPatch);
        end
    end
    
    methods (Access = private)
        
        function ap = updateHistory(obj, numActors)
            ap = obj.ActorPatches;
            create = ~all(ishghandle(ap(:))) || numActors~=size(ap,1);
            if create
                hAxes = obj.ParentPlot.Parent;
                set(hAxes.Legend, 'AutoUpdate', 'on');
                delete(ap(ishghandle(ap(:))));
                set(hAxes.Legend, 'AutoUpdate', 'off');
                ap = matlab.graphics.primitive.Patch.empty;
            elseif obj.HistoryDepth > 0
                for i=1:numel(ap)
                    ap(i).FaceAlpha = abs(ap(i).FaceAlpha - obj.FaceAlpha/obj.HistoryDepth);
                end
                obj.pHistoryIndex = obj.pHistoryIndex + 1;
                if obj.pHistoryIndex > obj.HistoryDepth+1
                    obj.pHistoryIndex = 1;
                end
            end
        end
        
        function plotActorPatches(obj, position, varargin)
            hAxes = obj.ParentPlot.Parent;
            if ishghandle(hAxes)
                % get patches updating history with dimmed alpha
                ap = updateHistory(obj, size(position,1));

                % add/update column with new positioning.
                iHistory = obj.pHistoryIndex;
                
                ap = driving.birdsEyePlot.internal.plotActorPatches(hAxes, ap, iHistory, obj.FaceAlpha, position, varargin{:});
                set(ap,'LineStyle',obj.LineStyle);
                set(ap,'LineWidth',2);
                obj.ActorPatches = ap;
            end
        end
        
        function validateOutlines(~, position, rYaw, rLength, rWidth, originOffset, color)
            n = size(position,1);
            validateattributes(position,{'numeric'},{'real','2d','finite','ncols',2}, ...
                'plotOutline','position');
            if ~isempty(rYaw)
                validateattributes(rYaw,{'numeric'},{'real','vector','finite'}, ...
                    'plotOutline','yaw');
            end
            if ~isempty(rLength)
                validateattributes(rLength,{'numeric'},{'real','vector','finite'}, ...
                    'plotOutline','length');
            end
            if ~isempty(rWidth)
                validateattributes(rWidth,{'numeric'},{'real','vector','finite'}, ...
                    'plotOutline','width');
            end
            validateattributes(originOffset,{'numeric'},{'real','2d','finite','ncols',2}, ...
                'plotOutline','originOffset');
            validateattributes(color,{'numeric'},{'real','2d','finite','ncols',3}, ...
                'plotOutline','color');
            if n~=numel(rYaw)
                error(message('driving:birdsEyePlot:YawMismatch'));
            end
            if n~=numel(rLength)
                error(message('driving:birdsEyePlot:LengthMismatch'));
            end
            if n~=numel(rWidth)
                error(message('driving:birdsEyePlot:WidthMismatch'));
            end
            if n~=size(originOffset,1)
                error(message('driving:birdsEyePlot:OriginOffsetMismatch'));
            end
            if n~=size(color,1)
                error(message('driving:birdsEyePlot:ColorMismatch'));
            end
        end
    end
end