function helperPlotAssignmentMetrics(motMetrics,ggiwphdMetrics,gmphdMetrics)
    % This is a helper function and may be modified or removed in a future
    % release
    
    % Copyright 2019 The MathWorks, Inc.
    
    % New figure
    figure('Units','normalized','Position',[0.1 0.1 0.5 0.5]);
    
    % Extract data
    motTgt = ~isnan(motMetrics.AssignedTruthID);
    ggiwphdTgt = ~isnan(ggiwphdMetrics.AssignedTruthID);
    gmphdTgt = ~isnan(gmphdMetrics.AssignedTruthID);
    tgtTracks = [sum(motTgt) sum(ggiwphdTgt) sum(gmphdTgt)];
    tgtTrackIDs = {string(motMetrics.TrackID(motTgt));string(ggiwphdMetrics.TrackID(ggiwphdTgt));string(gmphdMetrics.TrackID(gmphdTgt))};
    
    motRedundant = motMetrics.RedundancyStatus | (~motMetrics.Surviving & motMetrics.RedundancyCount > 0);
    ggiwphdRedundant = ggiwphdMetrics.RedundancyStatus | (~ggiwphdMetrics.Surviving & ggiwphdMetrics.RedundancyCount > 0);
    gmphdRedundant = gmphdMetrics.RedundancyStatus | (~gmphdMetrics.Surviving & gmphdMetrics.RedundancyCount > 0);
    numRedundantTracks = [sum(motRedundant) sum(ggiwphdRedundant) sum(gmphdRedundant)];
    redTrackIDs = {string(motMetrics.TrackID(motRedundant));string(ggiwphdMetrics.TrackID(ggiwphdRedundant));string(gmphdMetrics.TrackID(gmphdRedundant))};
    
    motFalse = ~(motTgt | motRedundant);
    ggiwphdFalse = ~(ggiwphdTgt | ggiwphdRedundant);
    gmphdFalse = ~(gmphdTgt | gmphdRedundant);
    
    numFalseTracks = [sum(motFalse) sum(ggiwphdFalse) sum(gmphdFalse)];
    falseTrackIDs = {string(motMetrics.TrackID(motFalse));string(ggiwphdMetrics.TrackID(ggiwphdFalse));string(gmphdMetrics.TrackID(gmphdFalse))};
        
    
    data = [tgtTracks;numFalseTracks;numRedundantTracks];
    h = bar(1:2:6,data);
    motTracks = [tgtTrackIDs(1);falseTrackIDs(1);redTrackIDs(1)];
    phdTracks = [tgtTrackIDs(2);falseTrackIDs(2);redTrackIDs(2)];
    protoTracks = [tgtTrackIDs(3);falseTrackIDs(3);redTrackIDs(3)];
    
    for i = 1:3
        text(h(1).XData(i) + h(1).XOffset, h(1).YData(i)/2, strcat('T',motTracks{i}),'HorizontalAlignment','center', 'VerticalAlignment','middle','FontSize',14,'Color',[1 1 1],'FontWeight','bold');
    end
    
    for i = 1:3
        text(h(2).XData(i) + h(2).XOffset, h(2).YData(i)/2, strcat('T',phdTracks{i}),'HorizontalAlignment','center', 'VerticalAlignment','middle','FontSize',14,'Color',[1 1 1],'FontWeight','bold');
    end
    
    for i = 1:3
        text(h(3).XData(i) + h(3).XOffset, h(3).YData(i)/2, strcat('T',protoTracks{i}),'HorizontalAlignment','center', 'VerticalAlignment','middle','FontSize',14,'Color',[1 1 1],'FontWeight','bold');
    end
    
    ylabel('Number of Tracks');
    xlabel('Track Type');
    xticklabels({'Target Tracks','False Tracks', 'Redundant Tracks'});
    legend('Point Target Tracker','GGIW-PHD Tracker','Rectangular GM-PHD Tracker');
end