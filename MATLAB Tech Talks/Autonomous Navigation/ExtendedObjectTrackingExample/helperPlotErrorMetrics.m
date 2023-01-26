function helperPlotErrorMetrics(motMetrics,ggiwphdMetrics,gmphdMetrics)
    % This is a helper function and may be modified or removed in a future release.
    
    % Copyright 2019 The MathWorks, Inc
    
    % New figure
    hFigure = figure('Position',[0 0 1152 720]);
    movegui(hFigure, 'center');
    
    % Extract data from metrics
    catgs = categorical(motMetrics.TruthID);
    positionErrors = [motMetrics.PositionError ggiwphdMetrics.PositionError gmphdMetrics.PositionError];
    velocityErrors = [motMetrics.VelocityError ggiwphdMetrics.VelocityError gmphdMetrics.VelocityError];
    dimErrors = [motMetrics.DimensionsError ggiwphdMetrics.DimensionsError gmphdMetrics.DimensionsError];
    yawErrors = [motMetrics.YawError ggiwphdMetrics.YawError gmphdMetrics.YawError];
    
    % Position error plot
    subplot(2,2,1);
    bar(catgs,positionErrors);
    title('Position Error')
    ylabel('Error (m)');
    xlabel('Truth ID');
    l = legend('Point Target Tracker','GGIW-PHD Tracker','Rectangular GM-PHD Tracker');
    l.Orientation = 'horizontal';
    l.Position(1) = 0.35;
    l.Position(2) = 0.95;
          
    % Velocity error plot
    subplot(2,2,2);
    bar(catgs,velocityErrors);
    title('Velocity Error')
    ylabel('Error (m/s)');
    xlabel('Truth ID');
    
    % Dimension error plot
    subplot(2,2,3);
    bar(catgs,dimErrors);
    title('Dimension Error')
    ylabel('Error (m)');
    xlabel('Truth ID');
    
    % Yaw error plot
    subplot(2,2,4);
    bar(catgs,yawErrors);
    title('Yaw Error')
    ylabel('Error (deg)');
    xlabel('Truth ID');
end