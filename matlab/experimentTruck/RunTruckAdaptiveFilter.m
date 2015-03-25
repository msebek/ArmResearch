function [results] = RunTruckAdaptiveFilter( filter, odometry, observations, covEstimators )
% Returns a struct with fields:
%   means - filter means
%   covariances - filter covariances
%   innovations - filter innovations
%   residuals - filter residuals
%   prioriCovariances - filter innovation priori term H(P-)H
%   posterioriCovariances - filter residual posteriori term H(P+)H
%   adaptedR - Regressed matrices

nObs = numel( observations );
T = numel( odometry );
zDim = 2;

results.means = zeros(3,T);
results.covariances = zeros(3,3,T);
results.innovations = zeros(zDim,nObs);
results.residuals = zeros(zDim,nObs);
results.prioriCovariances = zeros(zDim,zDim,nObs);
results.posterioriCovariances = zeros(zDim,zDim,nObs);
results.adaptedR = zeros(zDim,zDim,nObs);

oInd = 1;
for tInd = 1:T
    
    t = odometry(tInd).time;
    
    while oInd <= numel( observations ) && observations( oInd ).time == t
        
        obs = observations(oInd);
        obsFunc = @(x) TruckObservation( x, [obs.landmark_x; obs.landmark_y] );
        obsJacFunc = @(x) TruckObservationJacobian( x, [obs.landmark_x; obs.landmark_y] );
        filter.SetObservationFunctions( obsFunc, obsJacFunc );
        
        Rt = covEstimators(obs.id).GetCovariance();

        % Priori computations
        [innovation, C, residual, Cv] = filter.Update( [obs.range; obs.bearing], Rt );
        covEstimators(obs.id).Update( residual,  Cv );

        results.innovations(:,oInd) = innovation;
        results.residuals(:,oInd) = residual;
        results.prioriCovariances(:,:,oInd) = C;
        results.posterioriCovariances(:,:,oInd) = Cv;
        results.adaptedR(:,:,oInd) = Rt;
        
        oInd = oInd + 1;
    end
    
    results.means(:,tInd) = filter.x;
    results.covariances(:,:,tInd) = filter.S;
    
    filter.Predict( odometry(tInd).H );
    
end