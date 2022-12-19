function [mask,blobSize,blobX] = ExampleHelperSignFollowingProcessImg(img, th)
% Performs vision tasks for sign tracking example
% Copyright 2019 The MathWorks, Inc.

% Initialize values
numColors = size(th,2);
blobSize = zeros(numColors,1);
blobX = -1*ones(numColors,1);
mask = zeros(size(img,1),size(img,2));

% Loop through each color
for colorIdx = 1:numColors
    
    % Color thresholding
    cMask = (img(:,:,1) >= th(1,colorIdx)) & (img(:,:,1) <= th(2,colorIdx)) & ...
            (img(:,:,2) >= th(3,colorIdx)) & (img(:,:,2) <= th(4,colorIdx)) & ...
            (img(:,:,3) >= th(5,colorIdx)) & (img(:,:,3) <= th(6,colorIdx));
    mask(cMask) = 1;
    
    % Approximate blob analysis
    % NOTE: To get more accurate results, you can alternatively use the 
    % vision.BlobAnalysis object in Computer Vision Toolbox
    [~,cols] = find(cMask);
    if ~isempty(cols)
        % Average the column value of all nonzero mask pixels to 
        % approximate object location.
        blobX(colorIdx) = mean(cols);
        % Approximate the size using the number of nonzero mask pixels
        blobSize(colorIdx) = sqrt(2*numel(cols));
    end

end
