%% Generates datasets of 2D cart observing a sequence of beacons
% Experiment Parameters
% We use an X-forward, Y-left
% The target starts in front of the observer, aligned with the observer
% The observer starts at the origin and moves around there

% trialRoot = 'C:/Users/Humhu/Dropbox/Research/Papers/rss2015/data/beaconTrialsCross/';
trialRoot = '../data/beaconTrials/';

cartStartPose = SE2( [1; 0; 0], 'vector' );

% Waypoints to hit
[wx, wy, wt] = meshgrid( -5:2:5, -5:2:5, -pi:pi/4:pi );
numAllWaypoints = numel(wx);
allWaypoints = SE2( [wx(:)'; wy(:)'; wt(:)'], 'vector' );

numWaypoints = 5;

% Generate the trajectory
trajParams.errScale = eye(3);
trajParams.gainMatrix = 5*diag( [1, 0, 5] );
trajParams.rangeThreshold = 0.1;
trajParams.turnThreshold = 0.05;
trajParams.errThreshold = 0.1;
trajParams.maxVelocity = 3;
trajParams.dt = 0.1;
trajParams.transitionCovariance = 1E-3*eye(3);

% Beacon sweep parameters
minBeacons = 2;
beaconStep = 1;
maxBeacons = 5; % Runs out of memory at 10

% Generate beacon positions
[beaconX, beaconY] = meshgrid( -6:2:6, -6:2:6 );
numAllBeacons = numel( beaconX );
allBeacons = [ beaconX(:)'; beaconY(:)' ];

beaconNums = minBeacons:beaconStep:maxBeacons;
nbCell = num2cell( beaconNums );
%niCell = num2cell( [8,20,32,48] ); % Doing 4-fold validation

% 10 datapoints per parameter
niCell = num2cell( [4,12,24,48] );

% 20 datapoints per parameter
niCell = num2cell( [8,24,52,96] );

testParams = struct( 'nBeacons', nbCell, 'nIters', niCell );

nParamSets = numel( testParams );
for i = 1:nParamSets
    params = testParams(i);
    
    for iter = 1:params.nIters
        % Resample beacons for each trial
        trialBeaconInds = randperm(numAllBeacons, params.nBeacons);
        trialBeacons = allBeacons(:,trialBeaconInds);
        
        trialWaypointInds = randperm(numAllWaypoints, numWaypoints);
        trialWaypoints = allWaypoints(trialWaypointInds);
        
        trialName = ['BeaconCartSim_', num2str(params.nBeacons), 'Beacons_Trial', num2str(iter)];
        fprintf( ['Running trial ', trialName, '\n'] );
        GenerateBeaconTrial( cartStartPose, trialWaypoints, trajParams, trialBeacons, [trialRoot, trialName, '.mat'] );
    end
end

%% We want to scale each feature to [-1,1] roughly
datasetScales = cell(1,nParamSets);
numKeys = 1;

for i = 1:nParamSets
    
    params = testParams(i);
    
    % To get fdim
    trialName = [trialRoot, 'BeaconCartSim_', num2str(params.nBeacons), 'Beacons_Trial1.mat'];
    trial = load( trialName );
    fDim = size( trial.trueFeatures, 1 );
    featureScales = zeros(fDim,1);
    
    for iter = 1:params.nIters
        trialName = ['BeaconCartSim_', num2str(params.nBeacons), 'Beacons_Trial', num2str(iter)];
        fprintf( ['Parsing trial ', trialName, '\n'] );
        trial = load( [trialRoot, trialName, '.mat'] );
        trialScales = max( abs(trial.trueFeatures), [], 2 );
        featureScales( trialScales > featureScales ) = trialScales( trialScales > featureScales );
    end

    datasetScales{i} = featureScales;
    numKeys = numKeys + 1;
end

[testParams.featureScales] = deal( datasetScales{:} );

save( [trialRoot, 'BeaconDatasetParams.mat'], 'testParams' );

