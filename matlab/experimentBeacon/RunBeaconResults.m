function [results] = RunBeaconResults( trials, regressor )
% Returns aggregated results in struct with fields:
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
%       predictedObss - Predicted observations
%   true - struct with true values in fields:
%       poses - True poses
%       errors - True observation noise samples
%       features - True features;
%       R - Regressed observation covariance
%       obs - True observations
%   difference - Function handle to subtract states correctly

numTrials = numel(trials);
results = [];

fDiff = @(a,b) [ a(1:2,:) - b(1:2,:); wrapToPi(a(3,:) - b(3,:)) ];
results.difference = fDiff;

for i = 1:numTrials
    
    trial = trials(i);
    
    % Create and run filter
    % TODO: Filter initialization
    ekfTransFunc = @(x,u) BeaconCartTransition(x, u, trial.trajParams.dt);
    ekfObsFunc = @(x) BeaconCartObservation(x, trial.beacons);
    ekfObsJacFunc = @(x) BeaconCartObservationJacobian(x, trial.beacons );
    filter = ExtendedKalmanFilter( ekfTransFunc, @BeaconCartTransitionJacobian, ...
        ekfObsFunc, ekfObsJacFunc, @BeaconCartObservationDifference );
    
    filter.Initialize( trial.cartPoses(1).GetVector(), 1E-3*eye(3) );
    filter.SetTransitionCovariance( 1E-3*eye(3) );
    fRes = RunBeaconFilter( filter, trial.cartControls, trial.noisedObservations, regressor );
    
    if isfield( results, 'filter' )
        results.filter = AppendStructs( results.filter, fRes );
    else
        results.filter = fRes;
    end
    
    % Create and run smoother
    % TODO: Smoother initialization
    
    smoother = ExtendedKalmanSmoother( ekfTransFunc, @BeaconCartTransitionJacobian, ...
        ekfObsFunc, ekfObsJacFunc, fDiff, @BeaconCartObservationDifference );
    smoother.Initialize( trial.cartPoses(1).GetVector(), 1E-3*eye(3) );
    smoother.SetTransitionCovariance( 1E-3*eye(3) );
    sRes = RunBeaconSmoother( smoother, trial.cartControls, trial.noisedObservations, regressor );
    
    if isfield( results, 'smoother' )
        results.smoother = AppendStructs( results.smoother, sRes );
    else
        results.smoother = sRes;
    end
    
    true.poses = trial.cartPoses.GetVector();
    true.errors = trial.noiseValues;
    true.features = trial.trueFeatures;
    true.R = Regress( regressor, true.features );
    true.obs = trial.noisedObservations;
    if isfield( results, 'true' )
        results.true = AppendStructs( results.true, true );
    else
        results.true = true;
    end
    
end