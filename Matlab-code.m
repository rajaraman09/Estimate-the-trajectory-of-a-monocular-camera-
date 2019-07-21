
images = imageDatastore(fullfile(toolboxdir('vision'), 'Trajectory', ...
    'Assignment'));

cam_mat = [1111 0 320; 0 615 240; 0 0 1];
cameraParams = cameraParameters('IntrinsicMatrix', cam_mat);
set1 = viewSet;

Irgb = readimage(images, 5);
player = vision.VideoPlayer;
step(player, Irgb);
%figure, imshow(X,map), figure, imshow(X,gmap);
prevI = undistortImage(rgb2gray(Irgb), cameraParams); 
prevPoints = detectSURFFeatures(prevI, 'MetricThreshold', 500);

numPoints = 150;
prevPoints = selectUniform(prevPoints, numPoints, size(prevI));
%figure, imshow(I), figure, imshow(J);
prevFeatures = extractFeatures(prevI, prevPoints, 'Upright', true);
viewId = 1;
set1 = addView(set1, viewId, 'Points', prevPoints, 'Orientation', eye(3),...
    'Location', [0 0 0]);
% Setup axes.
figure
axis([-220, 50, -140, 20, -50, 300]);

% Set Y-axis to be vertical pointing down.
view(gca, 3);
set(gca, 'CameraUpVector', [0, -1, 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);

grid on
hold on

% Plot estimated camera pose. 
cameraSize = 7;
camEstimated = plotCamera('Size', cameraSize, 'Location',...
    set1.Views.Location{1}, 'Orientation', set1.Views.Orientation{1},...
    'Color', 'r', 'Opacity', 0);

% Plot actual camera pose.
camActual = plotCamera('Size', cameraSize, 'Location', ...
    groundTruthPoses.Location{1}, 'Orientation', ...
    groundTruthPoses.Orientation{1}, 'Color', 'r', 'Opacity', 0);

% Initialize camera trajectories.
trajectroy_esti = plot3(0, 0, 0, 'g-');
trajectoryActual    = plot3(0, 0, 0, 'b-');

title('Trajectory');
% Read and display the image.
viewId = 2;
Irgb = readimage(images, viewId);
step(player, Irgb);

% Convert to gray scale and undistort.
I = undistortImage(rgb2gray(Irgb), cameraParams);

% Match features between the previous and the current image.
[currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
    prevFeatures, I);

% Estimate the pose of the current view relative to the previous view.
[orient, loc, inlierIdx] = helperEstimateRelativePose(...
    prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), cameraParams);

% Exclude epipolar outliers.
indexPairs = indexPairs(inlierIdx, :);
    
% Add the current view to the view set.
set1 = addView(set1, viewId, 'Points', currPoints, 'Orientation', orient, ...
    'Location', loc);
% Store the point matches between the previous and the current views.
set1 = addConnection(set1, viewId-1, viewId, 'Matches', indexPairs);
set1 = helperNormalizeViewSet(set1, groundTruthPoses);
helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(set1), ...
    groundTruthPoses);
helperUpdateCameraTrajectories(viewId, trajectroy_esti, trajectoryActual,...
    poses(set1), groundTruthPoses);
prevI = I;
prevFeatures = currFeatures;
prevPoints   = currPoints;
for viewId = 3:15
    % Read and display the next image
    Irgb = readimage(images, viewId);
    step(player, Irgb);
    
    % Convert to gray scale and undistort.
    I = undistortImage(rgb2gray(Irgb), cameraParams);
    
    % Match points between the previous and the current image.
    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
        prevFeatures, I);
      
    % Eliminate outliers from feature matches.
    inlierIdx = helperFindEpipolarInliers(prevPoints(indexPairs(:,1)),...
        currPoints(indexPairs(:, 2)), cameraParams);
    indexPairs = indexPairs(inlierIdx, :);
    
    % Triangulate points 
    [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(set1,...
        cameraParams, indexPairs, currPoints);
    
    warningstate = warning('off','vision:ransac:maxTrialsReached');
    
    % Estimate the world camera pose for the current view.
    [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, ...
        cameraParams, 'Confidence', 99.99, 'MaxReprojectionError', 0.8);
    
    % Restore the original warning state
    warning(warningstate)
    
    % Add the current view to the view set.
    set1 = addView(set1, viewId, 'Points', currPoints, 'Orientation', orient, ...
        'Location', loc);
    
    % Store the point matches between the previous and the current views.
    set1 = addConnection(set1, viewId-1, viewId, 'Matches', indexPairs);    
    
    tracks = findTracks(set1); % Find point tracks spanning multiple views.
        
    camPoses = poses(set1);    % Get camera poses for all views.
    
    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Refine camera poses using bundle adjustment.
    [~, camPoses] = bundleAdjustment(xyzPoints, tracks, camPoses, ...
        cameraParams, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,...
        'RelativeTolerance', 1e-9, 'MaxIterations', 300);
        
    set1 = updateView(set1, camPoses); % Update view set.
    set1 = helperNormalizeViewSet(set1, groundTruthPoses);
    helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(set1), ...
        groundTruthPoses);
    helperUpdateCameraTrajectories(viewId, trajectroy_esti, ...
        trajectoryActual, poses(set1), groundTruthPoses);
    
    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end
for viewId = 16:numel(images.Files)
   
    Irgb = readimage(images, viewId);
    step(player, Irgb);

    I = undistortImage(rgb2gray(Irgb), cameraParams);
    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
        prevFeatures, I);    
   
    [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(set1, ...
        cameraParams, indexPairs, currPoints);
    warningstate = warning('off','vision:ransac:maxTrialsReached');
    
    % Estimate the world camera pose for the current view.
    [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, ...
        cameraParams, 'MaxNumTrials', 5000, 'Confidence', 99.99, ...
        'MaxReprojectionError', 0.8);
    warning(warningstate)
    
    % Add the current view and connection to the view set.
    set1 = addView(set1, viewId, 'Points', currPoints, 'Orientation', orient, ...
        'Location', loc);
    set1 = addConnection(set1, viewId-1, viewId, 'Matches', indexPairs);
    if mod(viewId, 7) == 0        
        % Find point tracks in the last 15 views and triangulate.
        windowSize = 15;
        startFrame = max(1, viewId - windowSize);
        tracks = findTracks(set1, startFrame:viewId);
        camPoses = poses(set1, startFrame:viewId);
        [xyzPoints, reprojErrors] = triangulateMultiview(tracks, camPoses, ...
            cameraParams);
        
        fixedIds = [startFrame, startFrame+1];
        idx = reprojErrors < 2;
        
        [~, camPoses] = bundleAdjustment(xyzPoints(idx, :), tracks(idx), ...
            camPoses, cameraParams, 'FixedViewIDs', fixedIds, ...
            'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,...
            'RelativeTolerance', 1e-9, 'MaxIterations', 300);
        
        set1 = updateView(set1, camPoses); % Update view set.
    end
    
    % Update camera trajectory plot.
    helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(set1), ...
        groundTruthPoses);    
    helperUpdateCameraTrajectories(viewId, trajectroy_esti, ...
        trajectoryActual, poses(set1), groundTruthPoses);    
    
    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end
%hold function determines whether new graphics objects are added to the graph or replace objects in the graph.
hold off