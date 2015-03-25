function [results] = RunTruckResults( trials, regressors )
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
numRegressors = numel( regressors );
results = [];

fDiff = @(a,b) [ a(1:2,:) - b(1:2,:); wrapToPi(a(3,:) - b(3,:)) ];

for i = 1:numRegressors
    results(i).difference = fDiff;
end

for i = 1:numTrials
    
    trial = trials(i);
    
    % Create and run filter
    % TODO: Filter initialization
    % Observation functions are set in RunTruckFilter since the landmark
    % changes at each observation
    ekfTransFunc = @TruckTransition;
    filter = ExtendedKalmanFilter( ekfTransFunc, @TruckTransitionJacobian, ...
        [], [], @TruckObservationDifference );
    
    initPose = [trial.truth(1).x; trial.truth(1).y; trial.truth(1).th];
    filter.Initialize( initPose, 1E-3*eye(3) );
    filter.SetTransitionCovariance( 1E-3*eye(3) );
    fRes = RunTruckFilter( filter, trial.odometry, trial.observations, regressors );
    
    for j = 1:numRegressors
        if isfield( results(j), 'filter' ) && ~isempty( results(j).filter )
            results(j).filter = AppendStructs( results(j).filter, fRes(j) );
        else
            results(j).filter = fRes(j);
        end
    end
    
    % Create and run smoother
    % TODO: Smoother initialization
    
    smoother = ExtendedKalmanSmoother( ekfTransFunc, @TruckTransitionJacobian, ...
        [], [], fDiff, @BeaconCartObservationDifference );
    smoother.Initialize( initPose, 1E-3*eye(3) );
    smoother.SetTransitionCovariance( 1E-3*eye(3) );
    sRes = RunTruckSmoother( smoother, trial.odometry, trial.observations, regressors );
    
    for j = 1:numRegressors
        if isfield( results(j), 'smoother' ) && ~isempty( results(j).smoother )
            results(j).smoother = AppendStructs( results(j).smoother, sRes(j) );
        else
            results(j).smoother = sRes(j);
        end
    end
    
    true.poses = [trial.truth.x; trial.truth.y; trial.truth.th];
    
    for j = 1:numRegressors
        obs = trial.observations( [trial.observations.id] == j );
        true.errors = [obs.range_error; obs.bearing_error];
        true.features = [obs.features];
        true.R = Regress( regressors(j), [obs.features] );
        true.obs = [obs.range; obs.bearing];

        if isfield( results(j), 'true' ) && ~isempty( results(j).true )
            results(j).true = AppendStructs( results(j).true, true );
        else
            results(j).true = true;
        end
    end
    
end