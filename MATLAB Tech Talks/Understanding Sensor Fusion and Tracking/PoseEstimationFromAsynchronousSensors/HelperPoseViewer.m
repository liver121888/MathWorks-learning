classdef HelperPoseViewer < matlab.System
%HelperPoseViewer - Orientation and position visualization
%
%   % EXAMPLE: Visualize ground truth and fused pose data.
%
%   estPosition = [0 0 0.5];
%   estOrientation = quaternion([30 15 10], 'eulerd', ...
%       'ZYX', 'frame');
%
%   truePosition = [0 0 0];
%   trueOrientation = quaternion(1,0,0,0);
%
%   viewer = HelperPoseViewer;
%   viewer(estPosition, estOrientation, truePosition, trueOrientation);

%   Copyright 2017-2021 The MathWorks, Inc.



    properties (Nontunable)
        NumInputs = 4;

        XPositionLimits = [-1, 1];
        YPositionLimits = [-1, 1];
        ZPositionLimits = [-1, 1];
        OrientationTitle = {'Orientation - Estimated', 'Orientation - Ground Truth'}
        
        ReferenceFrame = 'NED';
    end

    properties (Hidden)
        AppWindow;
    end

    properties (Access = private)
        pPositionViewer;
        pOrientationViewer = gobjects;
    end

    properties (Access = private, Constant)
        POSITION_VIEWER_POSITION = [0 0 0.5 1];
        ORIENTATION_VIEWER_POSITION = [0.5 0 0.5 1];
        BORDER_HEIGHT = 0.05;
    end

    methods
        % Constructor
        function obj = HelperPoseViewer(varargin)
            setProperties(obj,nargin,varargin{:});
            createUI(obj);
        end

        % Destructor
        function delete(obj)
            fig = obj.AppWindow;
            if (~isempty(fig) && ishghandle(fig))
                delete(fig);
            end
        end

        function show(obj)
            fig = obj.AppWindow;
            if (~isempty(fig) && ishghandle(fig))
                fig.Visible = 'on';
            end
        end

        function hide(obj)
            set(obj.AppWindow,'Visible','off');
        end
    end

    methods (Access = protected)
        function val = getNumInputsImpl(obj)
            val = obj.NumInputs;
        end

        function setupImpl(obj, varargin)
            % Pass properties down
            positionViewer = obj.pPositionViewer;
            positionViewer.XLimits = obj.XPositionLimits;
            positionViewer.YLimits = obj.YPositionLimits;
            positionViewer.ZLimits = obj.ZPositionLimits;
     
            show(obj);
        end

        function stepImpl(obj, varargin)
            for i = obj.NumInputs/2:-1:1
                positions{i} = varargin{2*i-1};
                obj.pOrientationViewer(i).Orientation = varargin{2*i};
            end
            obj.pPositionViewer(positions{:});
        end

        function createUI(obj)
            createAppWindow(obj);

            createPositionViewer(obj);
            createOrientationViewer(obj);
        end

        function createAppWindow(obj)
            fig = figure('Name', 'Pose Viewer', ...
                'NumberTitle', 'off', ...
                'DockControls','off', ...
                'Units', 'normalized', ...
                'OuterPosition', [0 0.25 0.5 0.5], ...
                'Visible', 'off', ...
                'HandleVisibility', 'on', ...
                'NextPlot', 'new', ...
                'IntegerHandle', 'off', ...
                'CloseRequestFcn', @(x,~)set(x,'Visible', 'off'));

            obj.AppWindow = fig;
        end
        
        function releaseImpl(obj)
            release(obj.pPositionViewer);
        end
        
        function resetImpl(obj)
            reset(obj.pPositionViewer);
        end
        
        function createPositionViewer(obj)
            obj.pPositionViewer = HelperPositionViewer('AppWindow', obj.AppWindow, ...
                'AxesPosition', obj.POSITION_VIEWER_POSITION, 'NumInputs', obj.NumInputs/2, ...
                'ReferenceFrame', obj.ReferenceFrame);
        end

        function createOrientationViewer(obj)
            numOrientationInputs = obj.NumInputs/2;
            if (numOrientationInputs == 1)
                ax = axes(obj.AppWindow, 'OuterPosition', obj.ORIENTATION_VIEWER_POSITION);
                obj.pOrientationViewer = poseplot(ax, obj.ReferenceFrame);
            else
                borderHeight = obj.BORDER_HEIGHT;
                currAxesPosition = obj.ORIENTATION_VIEWER_POSITION;
                axesHeight = (currAxesPosition(4)-borderHeight*(numOrientationInputs+1)) / numOrientationInputs;
                currAxesPosition(2) = currAxesPosition(2) + borderHeight;
                currAxesPosition(4) = axesHeight;
                titles = obj.OrientationTitle;
                for ii = 1:numOrientationInputs
                    parent = axes('Parent', obj.AppWindow, 'OuterPosition', currAxesPosition);
                    colorOrder = parent.ColorOrder;
                    colorIndex = ii;
                    numColors = size(colorOrder, 1);
                    while (colorIndex > numColors)
                        colorIndex = colorIndex - numColors;
                    end
                    color = colorOrder(colorIndex,:);
                    pp = poseplot(parent, obj.ReferenceFrame, "PatchFaceColor", color);
                    title(parent, titles{ii});
                    currAxesPosition(2) = currAxesPosition(2) + axesHeight + borderHeight;
                    obj.pOrientationViewer(ii,:) = pp;
                end
                syncRotationCallback(obj.pOrientationViewer);
            end
        end
    end
end

function syncRotationCallback(objs)
%SYNCROTATIONCALLBACK - link rotation to another set of axes
linkedPropsObject = linkprop([objs.Parent], {'XLim', 'YLim', 'ZLim', ...
    'CameraPosition', 'CameraTarget', 'CameraUpVector', 'CameraViewAngle'});
for i = 1:numel(objs)
    objs(i).Parent.UserData.pLinkedPropsObject = linkedPropsObject;
end
end
