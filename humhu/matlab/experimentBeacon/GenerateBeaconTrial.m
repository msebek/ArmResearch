function [] = GenerateBeaconTrial( cartStart, waypoints, trajParams, beacons, matName )
% Runs the trial and saves it to matName

nBeacons = size(beacons,2);

[cartPoses, cartControls, cartTimes] = ...
    Trajectory2DSimulator( cartStart, waypoints, trajParams );
T = numel( cartPoses );

fprintf( 'Generated trial of length %d\n', T );

% Multiplicative ranges from 0.75 to 1.25
% Gaussian noise has variance of 1E-2;
noiseParams.rangeBias = [0.85,1.15];
noiseParams.rangeVariance = [-0.1,0.1];
noiseParams.bearingVariance = 1E-2;
[noiselessObservations, noisedObservations, noiseValues] = ...
    RangeBearingSimulator( cartPoses, beacons, noiseParams );

trueFeatures = BeaconCartFeatures( noiselessObservations );

% Visualize the trajectory
visualizeOn = false;
if visualizeOn
    ah = PlotSE2( cartPoses );
    plot( ah, beacons(1,:), beacons(2,:), 'x' );
    axis equal;
end

save( matName, 'cartPoses', 'cartControls', 'cartTimes', 'noiselessObservations', ...
    'noisedObservations', 'noiseValues', ...
    'cartStart', 'waypoints', 'trajParams', 'beacons', 'trueFeatures' );