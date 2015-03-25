%% Example framework for testing a range-bearing EKF

%% 0. Trial generation parameters

% ==== 0.1 Generate a grid of waypoints and convert them to SE2 objects
% Note that waypoints are both position and orientation.
[wx, wy, wt] = meshgrid( -5:2:5, -5:2:5, -pi:pi/4:pi );
numAllWaypoints = numel(wx);
allWaypoints = SE2( [wx(:)'; wy(:)'; wt(:)'], 'vector' );

% ==== 0.2 Generate a grid of beacon locations
[beaconX, beaconY] = meshgrid( -6:2:6, -6:2:6 );
numAllBeacons = numel( beaconX );
allBeacons = [ beaconX(:)'; beaconY(:)' ];

% ==== 0.3 Set controller parameters for generating the robot trajectory

% The SE2 tangent velocity controller gains.
trajParams.gainMatrix = 5*diag( [1, 0, 5] );

% The Euclidean distance threshold for switching to turn-to-face behavior
% With the SE2 tangent velocity controller this should rarely be used
trajParams.rangeThreshold = 0.1;

% The Mahalanobis distance matrix for calculating the waypoint error norm
trajParams.errScale = eye(3);

% The error norm threshold for 'reaching' a waypoint
trajParams.errThreshold = 0.1;

% The cap on the robot combined angular and linear velocity
trajParams.maxVelocity = 3;

% The time step of the simulator
trajParams.dt = 0.1;

% The covariance for the zero-mean body velocity noise added at each time
% step. Note that these will actually be SE2 tangent velocities that are
% integrated, so very large noises may give you unexpected results!
trajParams.transitionCovariance = 1E-3*eye(3);

% ==== 0.4 Specify initial cart pose
% 'vector' specifies input as [x;y;theta]
cartStartPose = SE2( [1; 0; 0], 'vector' );

% ==== 0.5 Specify sim noise parameters
% Note See RangeBearingSimulator.m for more details
% Uniform noise range for correlated range multiplier
noiseParams.rangeBias = [0.85,1.15];

% Uniform noise range for individual range multiplier
noiseParams.rangeVariance = [-0.1,0.1];

% Gaussian variance for individual beacon additive
noiseParams.bearingVariance = 1E-2;


%% 1. Generate the trial

% ==== 1.1 Randomly pick the waypoints and beacons
numBeaconsToUse = 5;
beaconInds = randperm( numAllBeacons, numBeaconsToUse );
beacons = allBeacons(:,beaconInds);

numWaypointsToUse = 4;
waypointInds = randperm( numAllWaypoints, numWaypointsToUse );
waypoints = allWaypoints(waypointInds);

% ==== 1.2 Generate the trial trajectory
[cartPoses, cartControls, cartTimes] = ...
    Trajectory2DSimulator( cartStartPose, waypoints, trajParams );

% ==== 1.3 Generate the trial observations
[noiselessObservations, noisedObservations, noiseValues] = ...
    RangeBearingSimulator( cartPoses, beacons, noiseParams );

% ==== 1.4 Visualize the trial
% NOTE This will be updated to a fancier vis later
visfh = figure;
visah = axes( 'parent', visfh );

PlotSE2( visah, cartPoses, {'b-'} );
plot( visah, beacons(1,:), beacons(2,:), 'g^', 'markersize', 10, 'markerfacecolor', 'g');

%% 2. Run an Extended Kalman Filter (EKF)

% Transition function for robot
ekfTransFunc = @(x,u) BeaconCartTransition(x, u, trajParams.dt);

% Transition Jacobian for robot
ekfTransJacFunc = @BeaconCartTransitionJacobian;

% Observation function for robot
ekfObsFunc = @(x) BeaconCartObservation(x, beacons);

% Observation Jacobian for robot
ekfObsJacFunc = @(x) BeaconCartObservationJacobian(x, beacons );

% Manifold differencing function for observations
ekfObsDiffFunc = @BeaconCartObservationDifference;

% Instantiate the EKF
ekf = ExtendedKalmanFilter( ekfTransFunc, ekfTransJacFunc, ekfObsFunc, ...
    ekfObsJacFunc, ekfObsDiffFunc );

% Initialize the EKF
x0 = cartPoses(1).GetVector();
S0 = 1E-3*eye(3);
ekf.Initialize( x0, S0 );

% Set EKF transition covariance
ekf.SetTransitionCovariance( 1E-3*eye(3) );

% Set EKF observation covariance ( Tune this! )
ekf.SetObservationCovariance( eye( 2*numBeaconsToUse ) );

T = numel( cartPoses );
ekfPoses(T) = SE2;
for t = 1:T
    ekf.Update( noisedObservations(:,t) );
    ekfPoses(t) = SE2( ekf.x, 'vector' );
    if t ~= T
        ekf.Predict( cartControls(:,t) );
    end
end

% Visualize results
PlotSE2( visah, ekfPoses, {'r-'} );

%% 3. Run an Extended Kalman Smoother (EKS)

% Transition function for robot
eksTransFunc = @(x,u) BeaconCartTransition(x, u, trajParams.dt);

% Transition Jacobian for robot
eksTransJacFunc = @BeaconCartTransitionJacobian;

% Manifold differencing function for robot states
eksStateDiffFunc = @(a,b) [ a(1:2,:) - b(1:2,:); wrapToPi(a(3,:) - b(3,:)) ];

% Observation function for robot
eksObsFunc = @(x) BeaconCartObservation(x, beacons);

% Observation Jacobian for robot
eksObsJacFunc = @(x) BeaconCartObservationJacobian(x, beacons );

% Manifold differencing function for observations
eksObsDiffFunc = @BeaconCartObservationDifference;

% Instantiate the EKF
eks = ExtendedKalmanSmoother( eksTransFunc, eksTransJacFunc, eksObsFunc, ...
    eksObsJacFunc, eksStateDiffFunc, eksObsDiffFunc );

% Initialize the EKF
x0 = cartPoses(1).GetVector();
S0 = 1E-3*eye(3);
eks.Initialize( x0, S0 );

% Set EKF transition covariance
eks.SetTransitionCovariance( 1E-3*eye(3) );

% Set EKF observation covariance ( Tune this! )
eks.SetObservationCovariance( eye( 2*numBeaconsToUse ) );

T = numel( cartPoses );
for t = 1:T
    eks.ForwardUpdate( noisedObservations(:,t) );
    if t ~= T
        eks.ForwardPredict( cartControls(:,t) );
    end
end

eksSmoothedMeans = eks.SmoothAll();
eksPoses = SE2( eksSmoothedMeans, 'vector' );

% Visualize results
PlotSE2( visah, eksPoses, {'g-'} );

%% 4. Run the odometry only

% Transition function for robot
okfTransFunc = @(x,u) BeaconCartTransition(x, u, trajParams.dt);

% Transition Jacobian for robot
okfTransJacFunc = @BeaconCartTransitionJacobian;

% Instantiate the odometry-only filter
okf = ExtendedKalmanFilter( okfTransFunc, okfTransJacFunc, [], [], [] );

% Initialize the EKF
x0 = cartPoses(1).GetVector();
S0 = 1E-3*eye(3);
okf.Initialize( x0, S0 );

% Set EKF transition covariance
okf.SetTransitionCovariance( 1E-3*eye(3) );

T = numel( cartPoses );
okfPoses(T) = SE2;
for t = 1:T
    okfPoses(t) = SE2( okf.x, 'vector' );
    if t ~= T
        okf.Predict( cartControls(:,t) );
    end
end

PlotSE2( visah, okfPoses, {'k-'} );

