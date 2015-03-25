function [results] = RunTruckBaselines( trials, windowEstimators )
% Returns aggregated results for baselines in struct with fields:
%   adaptive - struct with windowed adaptive results in fields:      
%       means - Posteriori means at each step
%       covariances - Posteriori covariances at each step
%       innovations - Innovations at each step (pre update obs err)
%       residuals - Residuals at each step (post update obs err)
%       prioriCovariances - Mapped priori covariance HP(-)H
%       posterioriCovariances - Mapped posteriori covariance H(P+)H
%       adaptedR - Regressed observation covariance

numTrials = numel(trials);
results = [];

fDiff = @(a,b) [ a(1:2,:) - b(1:2,:); wrapToPi(a(3,:) - b(3,:)) ];
results.difference = fDiff;

for i = 1:numTrials
    
    trial = trials(i);
    
    % Create and run adaptive filter
    % TODO: Filter initialization
    ekfTransFunc = @TruckTransition;
    filter = ExtendedKalmanFilter( ekfTransFunc, @TruckTransitionJacobian, ...
        [], [], @TruckObservationDifference );
    
    initPose = [trial.truth(1).x; trial.truth(1).y; trial.truth(1).th];
    filter.Initialize( initPose, 1E-3*eye(3) );
    filter.SetTransitionCovariance( 1E-3*eye(3) );
    fRes = RunTruckAdaptiveFilter( filter, trial.odometry, trial.observations, windowEstimators );
    
    if isfield( results, 'adaptive' )
        results.adaptive = AppendStructs( results.adaptive, fRes );
    else
        results.adaptive = fRes;
    end
    
    true.poses = [trial.truth.x; trial.truth.y; trial.truth.th];
    true.errors = [trial.observations.range_error; trial.observations.bearing_error];
    true.features = [trial.observations.features];
    if isfield( results, 'true' )
        results.true = AppendStructs( results.true, true );
    else
        results.true = true;
    end
    
end