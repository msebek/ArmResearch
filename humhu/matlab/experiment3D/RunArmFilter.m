function [results] = RunArmFilter( filter, displacements, measurements, regressor, tag )
% Runs a Kalman filter to estimate the camera pose
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

% zPred = filter.x\tag.mean;
% feat = ArmFeatures( zPred );
T = numel( displacements );
% fDim = size( feat, 1 );

results.means = SE3.empty(1,0);
results.covariances = zeros(6,6,T);
results.innovations = zeros(6,T);
results.residuals = zeros(6,T);
results.prioriCovariances = zeros(6,6,T);
results.posterioriCovariances = zeros(6,6,T);
% results.prioriFeatures = zeros(fDim, T);
% results.posterioriFeatures = zeros(fDim, T);

for t = 1:T
    
    % Priori computations
%     zPred = filter.x\tag.mean; % Observation is directly tag relative to arm
%     feat = ArmFeatures( zPred );
%     Rt = Regress( regressor, feat );
    
%     armEstimateMean = tag.mean/measurements(t);
%     armEstimateAdjoint = armEstimateMean.GetAdjoint();
%     armEstimateCovariance = tag.covariance + armEstimateAdjoint*Rt*armEstimateAdjoint';
    
%     [innovation, C, residual, Cv] = filter.Update( armEstimateMean, armEstimateCovariance );
    
    results.means(t) = filter.x;
    results.covariances(:,:,t) = filter.S;
%     results.innovations(:,t) = innovation;
%     results.residuals(:,t) = residual;
%     results.prioriCovariances(:,:,t) = C;
%     results.posterioriCovariances(:,:,t) = Cv;
%     results.prioriFeatures(:,t) = feat;
%     results.prioriR(:,:,t) = Rt;
    
    % Posteriori computations
%     zPred = filter.x\tag.mean;
%     results.posterioriFeatures(:,t) = ArmFeatures( zPred );
%     results.posterioriR(:,:,t) = Regress( regressor, results.posterioriFeatures(:,t) );
    
    filter.Predict( displacements(t) );
    
end