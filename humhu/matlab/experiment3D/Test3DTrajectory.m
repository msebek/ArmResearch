%% Test routine for a target observed in 3D
% Experiment Parameters
% We use an X-forward, Y-left, Z-up convention
% The target starts in front of the observer, aligned with the observer
% The observer starts at the origin and moves around there

targetPos = [10;0;0];
targetQuat = euler2quat( [pi/2; 0; 0] ); % Yaw pitch roll
targetPose = SE3( [targetPos; targetQuat] );

observerStartPos = [0;0;0];
observerStartQuat = euler2quat( [0; 0; 0] );
observerStartPose = SE3( [observerStartPos; observerStartQuat] );

% Waypoints to hit
[wx, wy, wz] = meshgrid( -5:5:5, -5:5:5, -5:5:5 );
waypointPos = [wx(:)'; wy(:)'; wz(:)'];
waypointQuat = repmat( observerStartQuat, 1, size(waypointPos,2) );
waypointPoses = SE3( [waypointPos; waypointQuat] );

% Generate the trajectory
trajParams.errScale = eye(6);
trajParams.gainMatrix = 10*diag( [1, 1, 1, 1, 1, 1] );
trajParams.errThreshold = 1E-3;
trajParams.maxVelocity = 5;
trajParams.dt = 0.1;

[observerPoses, observerDisplacements, times] = ...
    Trajectory3DSimulator( observerStartPose, waypointPoses, trajParams );
T = numel( observerPoses );
[ah] = PlotSE3( observerPoses );
PlotSE3( ah, targetPose );
axis equal;
axis vis3d;

%% Execute a SE3 KF

% Noise is tracked on the left, corresponding to the end of the chain
S0 = eye(6);
kf = KalmanFilterSE3( observerStartPose, S0, 'left' );
estPoses = kf.x;
Q = 1E-3*eye(6);

for t = 1:T-1
    % Displacements compound on the right with noise on the left
    % This corresponds to noise at the end of the displacement
    kf.Predict( observerDisplacements(t), 'right', Q, 'left' );
    estPoses(t+1) = kf.x;
end