function [results] = RunBeaconBaselines( trials, windowEstimator )
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
    ekfTransFunc = @(x,u) BeaconCartTransition(x, u, trial.trajParams.dt);
    ekfObsFunc = @(x) BeaconCartObservation(x, trial.beacons);
    ekfObsJacFunc = @(x) BeaconCartObservationJacobian(x, trial.beacons );
    filter = ExtendedKalmanFilter( ekfTransFunc, @BeaconCartTransitionJacobian, ...
        ekfObsFunc, ekfObsJacFunc, @BeaconCartObservationDifference );
    
    filter.Initialize( trial.cartPoses(1).GetVector(), eye(3) );
    filter.SetTransitionCovariance( 1E-3*eye(3) );
    fRes = RunBeaconAdaptiveFilter( filter, trial.cartControls, trial.noisedObservations, windowEstimator );
    
    if isfield( results, 'adaptive' )
        results.adaptive = AppendStructs( results.adaptive, fRes );
    else
        results.adaptive = fRes;
    end
    
    true.poses = trial.cartPoses.GetVector();
    true.errors = trial.noiseValues;
    true.features = trial.trueFeatures;
    if isfield( results, 'true' )
        results.true = AppendStructs( results.true, true );
    else
        results.true = true;
    end
    
end