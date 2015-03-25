function [results] = RunBeaconAdaptiveFilter( filter, controls, observations, covEstimator )
% Returns a struct with fields:
%   means - filter means
%   covariances - filter covariances
%   innovations - filter innovations
%   residuals - filter residuals
%   prioriCovariances - filter innovation priori term H(P-)H
%   posterioriCovariances - filter residual posteriori term H(P+)H
%   adaptedR - Regressed matrices

T = size( observations, 2 );
zDim = size( observations, 1 );

results.means = zeros(3,T);
results.covariances = zeros(3,3,T);
results.innovations = zeros(zDim,T);
results.residuals = zeros(zDim,T);
results.prioriCovariances = zeros(zDim,zDim,T);
results.posterioriCovariances = zeros(zDim,zDim,T);
results.adaptedR = zeros(zDim,zDim,T);

for t = 1:T
    
    Rt = covEstimator.GetCovariance();
    
    % Priori computations
    [innovation, C, residual, Cv] = filter.Update( observations(:,t), Rt );
    covEstimator.Update( residual,  Cv );
    
    results.means(:,t) = filter.x;
    results.covariances(:,:,t) = filter.S;
    results.innovations(:,t) = innovation;
    results.residuals(:,t) = residual;
    results.prioriCovariances(:,:,t) = C;
    results.posterioriCovariances(:,:,t) = Cv;
    results.adaptedR(:,:,t) = Rt;
    
    % Have to do this since there is one less control than observation
    if t ~= T
        filter.Predict( controls(:,t) );
    end
    
end