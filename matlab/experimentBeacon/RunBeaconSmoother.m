function [results] = RunBeaconSmoother( smoother, controls, observations, regressor )
% Returns a struct with fields:
%   means - smoothed means
%   covariances - filter covariances
%   residuals - filter residuals
%   features - features at each smoothed state
%   R - Regressed matrices
%   predictedObs - Predicted observations

% Test to get fDim
zPred = smoother.h( smoother.x );
feat = BeaconCartFeatures( zPred );


T = size( observations, 2 );
zDim = size( observations, 1 );
fDim = size( feat, 1 );

results.means = zeros(3,T);
results.covariances = zeros(3,3,T);
results.residuals = zeros(zDim,T);
results.mappedCovariances = zeros(zDim,zDim,T);
results.features = zeros(fDim, T);
results.R = zeros(zDim,zDim,T);
results.predictedObs = zeros(zDim,T);

% Forward pass
for t = 1:T
    
    zPred = smoother.h( smoother.x );
    feat = BeaconCartFeatures( zPred );
    
    Rt = Regress( regressor, feat );
    
    smoother.ForwardUpdate( observations(:,t), Rt );
    smoother.ClampPosteriori();
    
    % Have to do this since there is one less control than observation
    if t ~= T
        smoother.ForwardPredict( controls(:,t) );
    end
    
end

[results.means, results.covariances] = smoother.SmoothAll();
results.residuals = zeros(zDim,T);
results.features = zeros(fDim,T);
for t = 1:T
    zPred = smoother.h( results.means(:,t) );
    results.residuals(:,t) = smoother.hDiff( observations(:,t), zPred );
    results.features(:,t) = BeaconCartFeatures( zPred );
    results.predictedObs(:,t) = zPred;
    
    Ht = smoother.H(results.means(:,t));
    results.mappedCovariances(:,:,t) = Ht*results.covariances(:,:,t)*Ht';
    
end
results.R = Regress( regressor, results.features );
