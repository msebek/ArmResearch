function [results] = RunTruckSmoother( smoother, odometry, observations, regressors )
% Returns a struct with fields:
%   means - smoothed means
%   covariances - filter covariances
%   residuals - filter residuals
%   features - features at each smoothed state
%   R - Regressed matrices
%   predictedObs - Predicted observations

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
    results(i).residuals = zeros(zDim,niObs);
    results(i).mappedCovariances = zeros(zDim,zDim,niObs);
    results(i).features = zeros(fDim, niObs);
    results(i).R = zeros(zDim,zDim,niObs);
    results(i).predictedObs = zeros(zDim,niObs);
    results(i).residuals = zeros(zDim,niObs);
    results(i).features = zeros(fDim,niObs);
end

% Forward pass
oInd = 1;
for tInd = 1:T
    
    t = odometry(tInd).time;
    
    while oInd <= numel(observations) && observations(oInd).time == t
        
        obs = observations(oInd);
        rInd = obs.id;
        
        obsFunc = @(x) TruckObservation( x, [obs.landmark_x; obs.landmark_y] );
        obsJacFunc = @(x) TruckObservationJacobian( x, [obs.landmark_x; obs.landmark_y] );
        smoother.SetObservationFunctions( obsFunc, obsJacFunc );
        
        zPred = smoother.h( smoother.x );
        feat = TruckFeatures( zPred );
        
        Rt = Regress( regressors(rInd), feat );
        
        smoother.ForwardUpdate( [obs.range; obs.bearing], Rt );
        
        oInd = oInd + 1;
    end
    smoother.ClampPosteriori();
    
    if tInd ~= T
        odom = odometry(tInd);
        smoother.ForwardPredict( odom.H );
    end
    
end

[smeans, scovs] = smoother.SmoothAll();
for i = 1:numRegressors
   results(i).means = smeans;
   results(i).covariances = scovs;
end

oInd = 1;
oiInds = ones(numRegressors,1);
for tInd = 1:T
    
    t = odometry(tInd).time;
    
    while oInd <= numel(observations) && observations(oInd).time == t
        
        obs = observations(oInd);
        rInd = obs.id;
        oiInd = oiInds(rInd);
        
        obsFunc = @(x) TruckObservation( x, [obs.landmark_x; obs.landmark_y] );
        obsJacFunc = @(x) TruckObservationJacobian( x, [obs.landmark_x; obs.landmark_y] );
        smoother.SetObservationFunctions( obsFunc, obsJacFunc );
        
        zPred = smoother.h( results(rInd).means(:,tInd) );
        results(rInd).residuals(:,oiInd) = smoother.hDiff( [obs.range; obs.bearing], zPred );
        feat = TruckFeatures( zPred );
        results(rInd).features(:,oiInd) = feat;
        results(rInd).R(:,:,oiInd) = Regress( regressors(rInd), feat );
        results(rInd).predictedObs(:,oiInd) = zPred;
        
        Ht = smoother.H(results(rInd).means(:,tInd));
        results(rInd).mappedCovariances(:,:,oiInd) = Ht*results(rInd).covariances(:,:,tInd)*Ht';
        
        oInd = oInd + 1;
        oiInds(rInd) = oiInds(rInd) + 1;
    end
end
