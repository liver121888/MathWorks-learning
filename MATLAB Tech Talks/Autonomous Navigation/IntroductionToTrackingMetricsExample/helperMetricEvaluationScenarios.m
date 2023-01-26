classdef helperMetricEvaluationScenarios < matlab.System
    % This is a helper class for "Introduction to Tracking Metrics"
    % example. It may be removed or modified in a future release.
    
    % Copyright 2019 The MathWorks, Inc.
    
    % Trajectories and Identities of each truth.
    properties (SetAccess = protected)
        TruthTrajectories
        TruthIdentities
        TrackTrajectories
        TrackIdentities
    end
    
    methods
        function obj = helperMetricEvaluationScenarios(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    
    methods (Access = protected)
        function setupImpl(obj,varargin)
           [truthTrajectories, truthIDs, trackTrajectories, trackIDs] = setupTrajectories(obj);
           obj.TruthTrajectories = truthTrajectories;
           obj.TruthIdentities = truthIDs;
           obj.TrackTrajectories = trackTrajectories;
           obj.TrackIdentities = trackIDs;
        end
        
        function [tracks, truths] = stepImpl(obj, time)
            tracks = getTracks(obj, time);
            truths = getTruths(obj, time);
        end
        
        function tracks = getTracks(obj, time)
            tracks = objectTrack.empty(0,1);
            for i = 1:numel(obj.TrackIdentities)
               traj = obj.TrackTrajectories{i};
               id = obj.TrackIdentities(i);
               [pos, orient, vel, acc, angVel] = lookupPose(traj,time);
               thisTrack = assembleTrack(obj,id,pos,orient,vel,acc,angVel);
               tracks = [tracks;thisTrack];    %#ok<AGROW>
            end
        end
        
        function truths = getTruths(obj,time)
            truths = [];
            for i = 1:numel(obj.TruthIdentities)
               traj = obj.TruthTrajectories{i};
               id = obj.TruthIdentities(i);
               [pos, orient, vel, acc, angVel] = lookupPose(traj,time);
               thisTruth = assembleTruth(obj,id,pos,orient,vel,acc,angVel);
               truths = [truths;thisTruth];    %#ok<AGROW>
            end
        end
        
        function track = assembleTrack(obj,id,pos,orient,vel,acc,angVel) %#ok<INUSL,INUSD>
            % Can switch between motion models here
            if ~isnan(pos)
                state = zeros(6,1);
                state(1:2:end) = pos;
                state(2:2:end) = vel;
                stateCov = eye(6);
                track = objectTrack('State',state,'StateCovariance',stateCov,...
                    'TrackID',id);
            else
                track = objectTrack.empty(0,1);
            end
        end
        
        function truth = assembleTruth(obj,id,pos,orient,vel,acc,angVel) %#ok<INUSL>
            if isnan(pos)
                pos = {};
                orient = {};
                id = {};
                vel = {};
                acc = {};
                angVel = {};
            end
            truth = struct('PlatformID',id,...
                   'Position',pos,...
                   'Velocity',vel,...
                   'Orientation',orient,...
                   'Acceleration',acc,...
                   'AngularVelocity',angVel);
           
        end
        
        function [truthTrajectories, truthIDs, trackTrajectories, trackIDs] = setupTrajectories(obj)
            % truthTrajectories
            truthTrajectories = cell(2,1);
            
            wps = [0 -2 0;10 -2 0];
            time = [0 10];
            truthTrajectories{1} = waypointTrajectory(wps,time);
            
            wps = [0 2 0;10 2 0];
            time = [0 10];
            truthTrajectories{2} = waypointTrajectory(wps,time);
            
            truthIDs = [1 2];
            
            % Tracks
            trackTrajectories = cell(0,1);
            
            % T1
            wps = [0.5 2 0;1 2 0;2 2 0;4 0 0];
            time = [0.5 1 2 4];
            trackTrajectories{end+1} = waypointTrajectory(addSomeNoise(wps),time);
            
            % T2
            wps = [0.5 -2 0;3 -2 0;5 -2 0];
            time = [0.5 3 5];
            trackTrajectories{end+1} = waypointTrajectory(addSomeNoise(wps),time);
            
            % T3
            wps = [0.5 3 0;2 2 0;3 2 0;4 2 0;5 2 0;6 2 0;7 2 0;8 2 0;8.25 1 0;8.5 0 0;8.75 -1 0;9 -2 0;10 -2 0];
            time = [0.5 2 3 4 5 6 7 8 8.25 8.5 8.75 9 10];
            trackTrajectories{end+1} = waypointTrajectory(addSomeNoise(wps),time);
            
            % T4
            wps = [6 -2 0;7 -2 0;8 -2 0;8.25 -1 0;8.5 0 0;8.75 1 0;9 2 0;10 2 0];
            time = [6 7 8 8.25 8.5 8.75 9 10];
            trackTrajectories{end+1} = waypointTrajectory(addSomeNoise(wps),time);
            
            % T5
            wps = [5 -3.5 0;6 -4.5 0];
            time = [8 9];
            trackTrajectories{end+1} = waypointTrajectory(addSomeNoise(wps),time);
            trackIDs = 1:5;
            
            for i = 1:numel(truthTrajectories)
                truthTrajectories{i}.SampleRate = 10;
            end
            
            for i = 1:numel(trackTrajectories)
                trackTrajectories{i}.SampleRate = 10;
            end
        end
    end
end

    
function wps = addSomeNoise(wps)
    wps(:,1:2) = wps(:,1:2) + 0.05*randn(size(wps,1),2);
end

