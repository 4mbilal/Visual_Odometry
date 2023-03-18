clear all
close all
clc

% h5disp('D:\RnD\Frameworks\Datasets\MidAir\PLE_trajectory_4022\sensor_records.hdf5');
% return
path = 'D:\RnD\Frameworks\Datasets\MidAir\Kite_trajectory_0008\';

gyr = h5read([path,'sensor_records.hdf5'],'/trajectory_0008/groundtruth/angular_velocity');
gyr = gyr*180/pi;
gz = gyr(3,:);
gz = gz;
% eul = quat2eul(a')*180/pi;
% subplot(1,3,1)
% plot(eul(:,1));
% subplot(1,3,2)
% plot(eul(:,2));
% subplot(1,3,3)
% plot(eul(:,3));
% figure
% return

fid = dir([path,'frames']);
Irgb = imread([fid(3).folder,'\',fid(3).name]);
Irgb = imresize(Irgb,0.25);


% Create the camera intrinsics object using camera intrinsics 
imageSize      = size(Irgb,[1,2]); % in pixels [mrows, ncols]
focalLength    = [imageSize(1)/2 imageSize(2)/2];        % specified in units of pixels
principalPoint = [imageSize(1)/2 imageSize(2)/2];        % in pixels [x, y]
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

thresh = 100;
prevI = undistortImage(im2gray(Irgb), intrinsics); 
prevPoints = detectSURFFeatures(prevI, 'MetricThreshold', thresh);
% prevPoints = detectSIFTFeatures(prevI);
% prevPoints   = detectORBFeatures(prevI,'ScaleFactor',1.2,'NumLevels',8);
% prevPoints   = detectHarrisFeatures(prevI);

numPoints = 300;
prevPoints = selectUniform(prevPoints, numPoints, size(prevI));
% Extract features. Using 'Upright' features improves matching quality if 
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(prevI, prevPoints);%, 'Upright', true);

t1 = 0;
t2 = 0;
errplot = [];
theta2D = 0;
theta3D = 0;
yaw_2D = [];
yaw_2D_q = [];
yaw_3D = [];
gyro_gt = [];
iter = 0;

for k=4:numel(fid)
    Irgb = imread([fid(k).folder,'\',fid(k).name]);
    Irgb = imresize(Irgb,0.25);
    I = im2gray(Irgb);
    
    % Match features between the previous and the current image.
    currPoints   = detectSURFFeatures(I, 'MetricThreshold', thresh);
%     currPoints = detectSIFTFeatures(I);
%     currPoints   = detectORBFeatures(I,'ScaleFactor',1.2,'NumLevels',8);
    currPoints   = selectUniform(currPoints, numPoints, size(I));
    currFeatures = extractFeatures(I, currPoints);%, 'Upright', true);
    indexPairs = matchFeatures(prevFeatures, currFeatures, 'Unique', true);
    currPointsI = currPoints.Location(indexPairs(:,2),:);
    prevPointsI = prevPoints.Location(indexPairs(:,1),:);

%     imshow(Irgb)
%     hold on
%     plot(currPoints,'ShowScale',false)
%     hold off
%     drawnow

%     continue

%     pause
    if(numel(indexPairs(:,1))>4)
tic
        [tform,inlierIdx] = estimateGeometricTransform2D(currPointsI,prevPointsI,'similarity');
        inlierDistorted = currPoints(inlierIdx,:);
        inlierOriginal = prevPoints(inlierIdx,:);
        invTform = invert(tform);
        Ainv = invTform.T;
        ss = Ainv(1,2);
        sc = Ainv(1,1);
        scaleRecovered = hypot(ss,sc);
        % Recover the rotation in which a positive value represents a rotation in
        % the clockwise direction.
        theta2D = atan2d(-ss,sc);
%         if(theta2D<-45)
%             theta2D = theta2D + 180;
%         end
%         if(theta2D>45)
%             theta2D = theta2D - 180;
%         end

t1 = toc*0.05 + t1*0.95;

tic
    % Estimate the pose of the current view relative to the previous view.
    [orient, loc, inlierIdx,err] = helperEstimateRelativePose1(prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), intrinsics);
t2 = toc*0.05 + t2*0.95;
    else
        err = 1;
    end
    if(err)
%         disp('.')
    else
        orient = rotm2eul(orient)*180/pi;
        theta3D = -orient(1);
        
        

    end
            yaw_2D = [yaw_2D theta2D*25];
            yaw_2D_q = [yaw_2D_q numel(indexPairs(:,1))];
            yaw_3D = [yaw_3D theta3D*25];
            gyro_gt = [gyro_gt gz(iter*4+1)];
            if(rem(iter,10)==0)
            plot(yaw_2D)
            hold on
%             plot(yaw_3D)
            plot(gyro_gt)
            hold off
            drawnow
        end
    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;
    iter = iter + 1;
end
figure
plot(yaw_2D)
figure
plot(yaw_3D)


function [orientation, location, inlierIdx, err] = ...
    helperEstimateRelativePose1(matchedPoints1, matchedPoints2, cameraParams)
err = 0;

if ~isnumeric(matchedPoints1)
    matchedPoints1 = matchedPoints1.Location;
end

if ~isnumeric(matchedPoints2)
    matchedPoints2 = matchedPoints2.Location;
end

% while(1)
%     [E, inlierIdx] = estimateEssentialMatrix(matchedPoints1, matchedPoints2,...
%         cameraParams,'Confidence',99,'MaxNumTrials',1000);
%     E
%     pause
% end
for i = 1:10
    % Estimate the essential matrix.    
    [E, inlierIdx] = estimateEssentialMatrix(matchedPoints1, matchedPoints2,...
        cameraParams,'Confidence',99,'MaxNumTrials',1000);
%     sum(inlierIdx) / numel(inlierIdx)
% keyboard
    % Make sure we get enough inliers
    if sum(inlierIdx) / numel(inlierIdx) < .1
        continue;
    end
    
    % Get the epipolar inliers.
    inlierPoints1 = matchedPoints1(inlierIdx, :);
    inlierPoints2 = matchedPoints2(inlierIdx, :);    
    
    % Compute the camera pose from the fundamental matrix. Use half of the
    % points to reduce computation.
    [orientation, location, validPointFraction] = ...
        relativeCameraPose(E, cameraParams, inlierPoints1(1:2:end, :),...
        inlierPoints2(1:2:end, :));

    % validPointFraction is the fraction of inlier points that project in
    % front of both cameras. If the this fraction is too small, then the
    % fundamental matrix is likely to be incorrect.
%     validPointFraction
    if validPointFraction > .8
       return;
    end
end

% After 100 attempts validPointFraction is still too low.
% error('Unable to compute the Essential matrix');
err = 1;
end
