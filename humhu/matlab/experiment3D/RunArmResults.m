function [results] = RunArmResults( trajectory, regressor, tag )
% Returns aggregated results in struct with fields:
%   tag - struct with tag estimation results in fields:
%       mean - tag pose in robot frame mean
%       covariance - uncertainty of mean estimate
%   filter - struct with filter results in fields:
%       means - Posteriori means at each step
%       covariances - Posteriori covariances at each step
%       innovations - Innovations at each step (pre update obs err)
%       residuals - Residuals at each step (post update obs err)
%       prioriCovariances - Mapped priori covariance HP(-)H
%       posterioriCovariances - Mapped posteriori covariance H(P+)H
%       prioriFeatures - Priori features (generated after predict step)
%       prioriR - Regressed observation covariance
%       posterioriFeatures - Posteriori features (generated after update)
%       posterioriR = Regressed observation covariance
%   smoother - struct with smoother results in fields:
%       means - Smoothed means at each step
%       covariances - Smoothed covariances at each step
%       residuals - Post-smoothing observation errors at each step
%       mappedCovariances - State covariance mapped HPH
%       features - Post-smoothing features
%       R - Regressed observation covariance
%   true - struct with true values in fields:
%       poses - True poses
%       errors - True observation noise
%       features - True features;
%       R - Regressed observation covariance
%   difference - Function handle to subtract states correctly

fDiff = @ArmDiffFunction;
results.difference = fDiff;

%% Create and run filter to estimate arm position using tag
armFilter = KalmanFilterSE3();
armFilter.Initialize( trajectory(1).armPose, 1E-3*eye(6) );
armFilter.SetTransitionCovariance( 1E-3*eye(6) );
results.filter = RunArmFilter( armFilter, [trajectory.tagMeasurement], [trajectory.noisedDisplacement], regressor, tag );

%% Create and run smoother to estimate arm position using tag
armSmoother = KalmanSmootherSE3();
armSmoother.Initialize( trajectory(1).armPose, 1E-3*eye(6) );
armSmoother.SetTransitionCovariance( 1E-3*eye(6) );
results.smoother = RunArmSmoother( armSmoother, [trajectory.tagMeasurement], [trajectory.noisedDisplacement], regressor, tag );

%% Get true fields
true.poses = [trajectory.armPose];
zPred = true.poses\tag.mean;
zErr = [trajectory.tagMeasurement]\zPred;
true.errors = zErr.GetCoordinates();
true.features = ArmFeatures( zPred );
true.R = Regress( regressor, true.features );
results.true = true;
