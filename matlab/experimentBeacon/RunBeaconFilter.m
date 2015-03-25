function [results] = RunBeaconFilter( filter, controls, observations, regressor )
% Returns a struct with fields:
%   means - filter means
%   covariances - filter covariances
%   innovations - filter innovations
%   residuals - filter residuals
%   prioriCovariances - filter innovation priori term H(P-)H
%   posterioriCovariances - filter residual posteriori term H(P+)H
%   prioriFeatures - features used during filter operation
%   prioriR - Regressed matrices
%   posterioriFeatures - features from posteriori states
%   posterioriR - Regressed matrices


% Test to get fDim
zPred = filter.h( filter.x );
feat = BeaconCartFeatures( zPred );


T = size( observations, 2 );
zDim = size( observations, 1 );
fDim = size( feat, 1 );

results.means = zeros(3,T);
results.covariances = zeros(3,3,T);
results.innovations = zeros(zDim,T);
results.residuals = zeros(zDim,T);
results.prioriCovariances = zeros(zDim,zDim,T);
results.posterioriCovariances = zeros(zDim,zDim,T);
results.prioriFeatures = zeros(fDim, T);
results.posterioriFeatures = zeros(fDim, T);

for t = 1:T
    
    % Priori computations
    zPred = filter.h( filter.x );
    feat = BeaconCartFeatures( zPred );
    Rt = Regress( regressor, feat );
    
    [innovation, C, residual, Cv] = filter.Update( observations(:,t), Rt );
    
    results.means(:,t) = filter.x;
    results.covariances(:,:,t) = filter.S;
    results.innovations(:,t) = innovation;
    results.residuals(:,t) = residual;
    results.prioriCovariances(:,:,t) = C;
    results.posterioriCovariances(:,:,t) = Cv;
    results.prioriFeatures(:,t) = feat;
    results.prioriR(:,:,t) = Rt;
    
    % Posteriori computations
    zPred = filter.h( filter.x );
    results.posterioriFeatures(:,t) = BeaconCartFeatures( zPred );
    results.posterioriR(:,:,t) = Regress( regressor, results.posterioriFeatures(:,t) );
    
    % Have to do this since there is one less control than observation
    if t ~= T
        filter.Predict( controls(:,t) );
    end
    
end