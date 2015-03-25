function [results] = RunTruckFilter( filter, odometry, observations, regressors )
% Returns a struct array with fields:
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
feat = TruckFeatures( [1;1] );

nObs = numel( observations );
T = numel( odometry );
zDim = 2;
fDim = size( feat, 1 );

numRegressors = numel(regressors);

results = [];
for i = 1:numRegressors
    niObs = sum( [observations.id] == i );
    results(i).means = zeros(3,niObs);
    results(i).covariances = zeros(3,3,niObs);
    results(i).innovations = zeros(zDim,niObs);
    results(i).residuals = zeros(zDim,niObs);
    results(i).prioriCovariances = zeros(zDim,zDim,niObs);
    results(i).posterioriCovariances = zeros(zDim,zDim,niObs);
    results(i).prioriFeatures = zeros(fDim, niObs);
    results(i).posterioriFeatures = zeros(fDim, niObs);
    results(i).prioriR = zeros(zDim,zDim,niObs);
end

oInd = 1;
oiInds = ones(numRegressors,1);
for tInd = 1:T
    
    t = odometry(tInd).time;
%     fprintf( 'Processing time %f\n', t );
    
    % Loop over observations
    while oInd <= nObs && observations(oInd).time == t
        
        obs = observations(oInd);
        rInd = obs.id;
        oiInd = oiInds(rInd);
        
        obsFunc = @(x) TruckObservation( x, [obs.landmark_x; obs.landmark_y] );
        obsJacFunc = @(x) TruckObservationJacobian( x, [obs.landmark_x; obs.landmark_y] );
        filter.SetObservationFunctions( obsFunc, obsJacFunc );
        
        % Priori computations
        zPred = filter.h( filter.x );
        feat = TruckFeatures( zPred );
        Rt = Regress( regressors(rInd), feat );
        
        [innovation, C, residual, Cv] = filter.Update( [obs.range; obs.bearing], Rt );
        results(rInd).innovations(:,oiInd) = innovation;
        results(rInd).residuals(:,oiInd) = residual;
        results(rInd).prioriCovariances(:,:,oiInd) = C;
        results(rInd).posterioriCovariances(:,:,oiInd) = Cv;
        results(rInd).prioriFeatures(:,oiInd) = feat;
        results(rInd).prioriR(:,:,oiInd) = Rt;
        
        % Posteriori computations
        zPred = filter.h( filter.x );
        feat =  TruckFeatures( zPred );
        results(rInd).posterioriFeatures(:,oiInd) = feat;
        results(rInd).posterioriR(:,:,oiInd) = Regress( regressors(obs.id), feat );
        
        oInd = oInd + 1;
        oiInds(rInd) = oiInds(rInd) + 1;
    end
    
    for rInd = 1:numRegressors
        results(rInd).means(:,tInd) = filter.x;
        results(rInd).covariances(:,:,tInd) = filter.S;
    end
        
    odom = odometry(tInd);
    filter.Predict( odom.H );
    
end