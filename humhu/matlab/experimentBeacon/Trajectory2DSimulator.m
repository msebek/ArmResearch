function [poses, controls, times] = Trajectory2DSimulator( start, waypoints, params )
% Simulates a cart that can only turn and drive forward. The cart dead
% reckons to the specified waypoints, meaning it will drift further and
% further for longer trials.
%
% start - An initial pose SE2 object
% waypoints - An ordered array of waypoint SE2 objects
% params - A trajectory generation parameter struct containing the fields:
%   gainMatrix - A 3x3 error controller gain matrix
%   rangeThreshold - The Euclidean distance threshold for turn-to-face
%   errScale - 3x3 Mahalanobis matrix for error norm
%   errThreshold - Waypoint 'reached' error threshold
%   maxVelocity - Maximum velocity allowed
%   dt - Simulation timestep
%   transitionCovariance - 3x3 body velocity noise covariance
%
% poses - Ordered array of true robot SE2 poses. poses(1) = start
% controls - Body velocity control outputs such that:
%        poses(i+1) = poses(i)*SE2( params.dt*control(:,i), 'twist' )
% times - Timestamps for poses and controls

currentTruePose = start;
currentBeliefPose = start;
poses = currentTruePose;
controls = zeros(2,1);
times = 0;

numWaypoints =  numel( waypoints );
waypointInd = 1;
ind = 1;

while( waypointInd <= numWaypoints )
    
    currentWaypoint = waypoints(waypointInd);
    
    err = currentBeliefPose\currentWaypoint;
    
    errVec = err.GetCoordinates();
    if errVec'*params.errScale*errVec < params.errThreshold
        waypointInd = waypointInd + 1;
%         fprintf( 'Reached waypoint %d\n', waypointInd-1 );
        continue;
    end
    
    rangeToWaypoint = norm( errVec(1:2) );
    
    if rangeToWaypoint < params.rangeThreshold
        control = [0; 0; errVec(3)];
    elseif errVec(1) < 1E-3
        s = sign( errVec(1) );
        if s == 0
            s = 1;
        end
        control = [errVec(1); 0; errVec(2)*s];
    else
        control = [errVec(1); 0; errVec(2)/errVec(1)];
    end

    control = params.gainMatrix*control;
    
    if norm(control) > params.maxVelocity
        control = control/norm(control)*params.maxVelocity;
    end
    
    % Inject some noise
    % Note that the noise enters as x(t+1) = x(t)*noise*displacement
    % to represent noise at the end of the displacement
    noiseVec = mvnrnd( zeros(1,3), params.transitionCovariance )';
    noiseDisplacement = SE2( noiseVec, 'twist' );
    
    % Belief pose propogates as odometry only
    % True pose tracks actual position perturbed by noise
    displacement = SE2( params.dt*control, 'twist' );
    currentTruePose = currentTruePose*noiseDisplacement*displacement;
    currentBeliefPose = currentBeliefPose*displacement;
    
    poses(ind+1) = currentTruePose;
    controls(:,ind) = control([1,3]);
    times(ind+1) = times(ind) + params.dt;
    ind = ind + 1;
    
end