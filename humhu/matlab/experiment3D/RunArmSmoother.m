function [results] = RunArmSmoother( smoother, displacements, measurements, regressor, tag )
% Returns a struct with fields:
%   means - smoothed means
%   covariances - filter covariances
%   residuals - filter residuals
%   features - features at each smoothed state
%   R - Regressed matrices

zPred = smoother.x\tag.mean;
feat = ArmFeatures( zPred );
T = numel( displacements );
fDim = size( feat, 1 );

results.means = [];
results.covariances = zeros(6,6,T);
results.residuals = zeros(6,T);
results.mappedCovariances = zeros(6,6,T);
results.features = zeros(fDim, T);
results.R = zeros(6,6,T);

% Forward pass
for t = 1:T
    
    zPred = smoother.x\tag.mean;
    feat = ArmFeatures( zPred );
    
    Rt = Regress( regressor, feat );
    
    armEstimateMean = tag.mean/measurements(t);
    armEstimateAdjoint = armEstimateMean.GetAdjoint();
    armEstimateCovariance = tag.covariance + armEstimateAdjoint*Rt*armEstimateAdjoint';
    
    smoother.Update( armEstimateMean, armEstimateCovariance );
    smoother.Predict( displacements(t) );
    
end

[results.means, results.covariances] = smoother.SmoothAll();
results.residuals = zeros(6,T);
results.features = zeros(fDim,T);
for t = 1:T
    zPred = results.means(t)\tag.mean;
    zErr = measurements(t)/zPred;
    results.residuals(:,t) = zErr.GetCoordinates();
    results.features(:,t) = ArmFeatures( zPred );
    
    results.mappedCovariances(:,:,t) = results.covariances(:,:,t);
    
end
results.R = Regress( regressor, results.features );
