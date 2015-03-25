%% Read the data
truthRaw = csvread('../data/truck/GroundTruth.csv', ',' )';
observationsRaw = csvread('../data/truck/LandmarksComplete.csv', ',' )';
odometryRaw = csvread('../data/truck/Odometry.csv', ',' )';

% Shift data to start at 0
t0 = max( [truthRaw(1,1), odometryRaw(1,1)] );
tf = min( [truthRaw(1,end), odometryRaw(1,end)] );
x0 = min( truthRaw(2,:) );
y0 = min( truthRaw(3,:) );

truthRaw(1,:) = truthRaw(1,:) - t0;
truthRaw(2,:) = truthRaw(2,:) - x0;
truthRaw(3,:) = truthRaw(3,:) - y0;

observationsRaw(1,:) = observationsRaw(1,:) - t0;
observationsRaw(5,:) = observationsRaw(5,:) - x0;
observationsRaw(6,:) = observationsRaw(6,:) - y0;

odometryRaw(1,:) = odometryRaw(1,:) - t0;

% First we resample the ground truth and odometry to be time aligned
% Both are 100 Hz by default, but there is a little bit of time variation
% and some dropped sections, it seems like
resampleDt = 0.01;
subsampleRatio = 0.33;
resampleTimes = 0:resampleDt:subsampleRatio*(tf-t0);
truthResampled = interp1( truthRaw(1,:), truthRaw(2:end,:)', resampleTimes )';
odometryResampled = interp1( odometryRaw(1,:), odometryRaw(2:end,:)', resampleTimes )';

% Assemble the data structs
truth = struct( 'time', num2cell( resampleTimes ), ...
    'x', num2cell( truthResampled(1,:) ), ...
    'y', num2cell( truthResampled(2,:) ), ...
    'th', num2cell( truthResampled(3,:) ) );

odomSE2 = SE2( [odometryResampled(1,:); zeros(1,numel(resampleTimes)); odometryResampled(2,:)]*resampleDt, 'twist' );
odometry = struct( 'time', num2cell( resampleTimes ), ...
    'linear_velocity', num2cell( odometryResampled(1,:) ), ...
    'angular_velocity', num2cell( odometryResampled(2,:) ), ...
    'H', squeeze( num2cell( odomSE2.GetTransform, [1,2] ) )' );

% Find the ground truth elements corresponding to the observation times
observationTruthInds = interp1( [truth.time], 1:numel(truth), observationsRaw(1,:), 'nearest' );
observationInBounds = ~isnan( observationTruthInds );
observationsRaw = observationsRaw(:,observationInBounds);
observationTruthInds = observationTruthInds(observationInBounds);

% Remap landmark IDs
[uniqueIDs, ~, remappedIDs] = unique( observationsRaw(4,:) );
remappedIDs = ones( size(observationsRaw,2), 1 ); % HACK~!!!
numLandmarks = max(remappedIDs);

observations = struct( 'time', num2cell( [truth(observationTruthInds).time] ), ...
    'range', num2cell( observationsRaw(2,:) ), ...
    'bearing', num2cell( observationsRaw(3,:) ), ...
    'id', num2cell( remappedIDs' ), ...
    'landmark_x', num2cell( observationsRaw(5,:) ), ...
    'landmark_y', num2cell( observationsRaw(6,:) ), ...
    'x', num2cell( [truth(observationTruthInds).x] ), ...
    'y', num2cell( [truth(observationTruthInds).y] ), ...
    'th', num2cell( [truth(observationTruthInds).th] ) );

%% Subsample the data
% subsampleRate = 1;
% subsampleDt = resampleDt*subsampleRate;
% subsampleInds = 1:subsampleRate:numel(resampleTimes);
% subsampleTimes = resampleTimes(subsampleInds);
%
% truth = truth(subsampleInds);
%
% odometrySubsampled = interp1( odometryRaw(1,:), odometryRaw(2:end,:)', subsampleTimes )';
% odomSE2 = SE2( [odometrySubsampled(1,:); zeros(1,numel(subsampleTimes)); odometrySubsampled(2,:)]*subsampleDt, 'twist' );
% odometry = struct( 'time', num2cell( subsampleTimes ), ...
%                    'linear_velocity', num2cell( odometrySubsampled(1,:) ), ...
%                    'angular_velocity', num2cell( odometrySubsampled(2,:) ), ...
%                    'H', squeeze( num2cell( odomSE2.GetTransform, [1,2] ) )' );
%
% observationMatches = ismember([observations.time], subsampleTimes);
% observations = observations(observationMatches);

%% Estimate measurements and remove outliers

landmark_pos = [ observations.landmark_x; observations.landmark_y ];
observation_pos = [ observations.x; observations.y; observations.th ];

rel_pos = bsxfun( @minus, landmark_pos, observation_pos(1:2,:) );
rEst = sqrt( sum( rel_pos.*rel_pos, 1 ) );
bEst = atan2( landmark_pos(2,:) - observation_pos(2,:), ...
    landmark_pos(1,:) - observation_pos(1,:) ) - observation_pos(3,:);
bEst = wrapToPi(bEst);

rErrs = rEst - [observations.range];
bErrs = wrapToPi(bEst - [observations.bearing]);

features = TruckFeatures( [rEst; bEst] );

rEstCell = num2cell( rEst );
[observations.true_range] = deal( rEstCell{:} );
bEstCell = num2cell( bEst );
[observations.true_bearing] = deal( bEstCell{:} );
rErrsCell = num2cell( rErrs );
[observations.range_error] = deal( rErrsCell{:} );
bErrsCell = num2cell( bErrs );
[observations.bearing_error] = deal( bErrsCell{:} );
featuresCell = num2cell( features, 1 );
[observations.features] = deal( featuresCell{:} );

rangeThreshold = 0.005; % Errors more than this distance are outliers
bearingThreshold = 0.04; % Errors more than this angle are outliers
obsBad = (abs(rErrs-0.4) > abs(rEst+30)*0.01 ) | (abs(bErrs) > bearingThreshold);

fprintf( 'Removed %d bad out of %d total observations.\n', sum(obsBad), numel(observations) );
observations = observations(~obsBad);

%% Plot errors

for i = 1:numLandmarks
    
    obs = observations( [observations.id] == i );
    
    figure;
    plot3( [obs.true_range], [obs.true_bearing], [obs.range_error], 'k.' );
    xlabel('Range r, (m)');
    ylabel('Bearing b, (rad)' );
    zlabel('Range error (m)');
    title( ['Landmark ', num2str(i), ' Range Errors'] );
    
    % figure;
    % hold on;
    % plot( rEst, rErrs, 'k.' );
    % plot( rEst, abs(rEst+30)*0.01, 'r-' );
    
    figure;
    plot3( [obs.true_range], [obs.true_bearing], [obs.bearing_error], 'k.' );
    xlabel('Range r, (m)');
    ylabel('Bearing b, (rad)' );
    zlabel('Bearing error (rad)');
    title( ['Landmark ', num2str(i), ' Bearing Errors'] );
    
%     figure;
%     plot( [observations.range_error], 'k.' );
%     ylabel( 'Range errs (m)' );
%     
%     figure;
%     plot(  [observations.bearing_error], 'k.' );
%     ylabel( 'Bearing errs (m)' );
end

%% Calculate feature scales and MLE covariance
featureScales = max( abs([observations.features]), [], 2 );
noiseCov = cov( [ [observations.range_error]; [observations.bearing_error] ]' );

%% Divide into folds
numFolds = 4;
timeSpan = truth(end).time - truth(1).time;
timeSplit = timeSpan/numFolds;

trues = cell(1,numFolds);
obs = cell(1,numFolds);
odoms = cell(1,numFolds);

% Need to avoid dropping t=0 data with greater or equal
for i = 1:numFolds
    if i == 1
        trues{i} = truth( [truth.time] >= (i-1)*timeSplit & [truth.time] <= i*timeSplit );
        obs{i} = observations( [observations.time] >= (i-1)*timeSplit & [observations.time] <= i*timeSplit );
        odoms{i} = odometry( [odometry.time] >= (i-1)*timeSplit & [odometry.time] <= i*timeSplit );
    else
        trues{i} = truth( [truth.time] > (i-1)*timeSplit & [truth.time] <= i*timeSplit );
        obs{i} = observations( [observations.time] > (i-1)*timeSplit & [observations.time] <= i*timeSplit );
        odoms{i} = odometry( [odometry.time] > (i-1)*timeSplit & [odometry.time] <= i*timeSplit );
    end
end

folds = struct( 'truth', trues, 'observations', obs, 'odometry', odoms );

dt = resampleDt;
save( '../data/truck/FoldedData.mat', 'folds', 'dt', 'featureScales', 'numLandmarks', 'noiseCov' );
