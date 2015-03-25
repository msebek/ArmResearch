function [poses, displacements, times] = Trajectory3DSimulator( start, waypoints, params )

currentPose = start;
poses = currentPose;
times = 0;

numWaypoints = numel( waypoints );
waypointInd = 1;
ind = 1;

while( waypointInd <= numWaypoints )
    
    currentWaypoint = waypoints(waypointInd);
    
    err = currentPose\currentWaypoint;

    errVec = err.GetCoordinates();
    if errVec'*params.errScale*errVec < params.errThreshold
        waypointInd = waypointInd + 1;
        fprintf( 'Reached waypoint %d\n', waypointInd-1 );
        continue;
    end
    
    control = params.gainMatrix*errVec;

    if norm(control) > params.maxVelocity
        control = control/norm(control)*params.maxVelocity;
    end
    
    displacement = SE3( params.dt*control );
    currentPose = currentPose*displacement;
    
    poses(ind+1) = currentPose;
    displacements(ind) = displacement;
    times(ind+1) = times(ind) + params.dt;
    ind = ind + 1;
    
end