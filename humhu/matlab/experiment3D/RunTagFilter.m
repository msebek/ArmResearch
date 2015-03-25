function [results] = RunTagFilter( filter, trajectory, regressor )
% Estimates the April tag position from the specified trajectory
% Returns the tag pose and uncertainty in the robot frame

N = numel( trajectory );

Q = 1E-3*eye(6); % Arm pose noise

for i = 1:N

    relPose = trajectory(i).cameraPose\filter.x;
    feature = ArmFeatures( relPose );
    R = Regress( regressor, feature );
        
    tagPoseMean = trajectory(i).cameraPose*trajectory(i).tagMeasurement(i);
    cameraPoseAdj = trajectory(i).cameraPose.GetAdjoint();
    tagPoseCov = cameraPoseAdj*R(:,:,i)*cameraPoseAdj' + Q;
    
    filter.Update( tagPoseMean, tagPoseCov );
    
end

results.mean = filter.x;
results.covariance = filter.S;