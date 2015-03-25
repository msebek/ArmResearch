% Tests GLI with a Kalman filter

%% Instantiate 1D 2nd order linear system
dt = 0.01;
A = [ 1, dt, 0;
    0, 1, dt;
    -10, 0, 0 ];
Aest = [ 1, dt, 0;
         0, 1, dt;
         -10, 0, 0 ];
C = [ 1, 0, 0;
    0, 1, 0 ];

%% Parameters
xDim = 3;
zDim = size(C,1);
fDim = 5;

vScale = 0.5;
wScale = -1;
wConst = 2;

% GLI dimensions for observation noise
vDim = (zDim -1 )*zDim/2; % Don't need terms for diagonal of L
wDim = zDim;

trainX0 = [0;3;0];
testX0 = [1;2;0];
trainT = 1000;
testT = 500;

Q = diag( [1E-3, 1E-3, 1E0] ); % Process noise
Qest = Q; % Process noise used in filters

% Covariance generating features are velocity, acceleration, squared terms
featureFunc = @(x) [ sqrt(abs(x(2,:))); 
                     abs(x(2,:))/10; 
                     sqrt(abs(x(3,:))); 
                     abs(x(3,:))/10; 
                     ones(1, size(x,2)) ];

% Random observation GLI parameters
%vTrue = vScale*rand( fDim, vDim ) - 0.5;
vTrue = zeros( fDim, vDim );
wTrue = [ 2, 2;
        0, 0;
        0, 0;
        0, 0;
        -3, -3];
% wTrue = [-1, -1;
%          0, 0;
%          0, 0;
%          0, 0;
%          1E-1, 1E-1];
trueParams.v = vTrue;
trueParams.w = wTrue;
trueParams.modelMode = 'covariance';
trueParams.dataMode = 'error';

% Gradient ascent parameters
vPenalty = 1E-3;
wPenalty = 1E-3;

ascentParams.stepSize = 1E-2;
ascentParams.backtrackAlpha = 0.1; % Improvement ratio
ascentParams.backtrackBeta = 0.8; % Step decrease ratio
ascentParams.gradientTolerance = 1E-4;
ascentParams.maximumIterations = 1000;
ascentParams.maximumBacktracks = 10;
ascentParams.sparsify = false;
ascentParams.showPlots = true;

vGuess = zeros( fDim, vDim );
wGuess = zeros( fDim, wDim );
% vGuess = rand( fDim, vDim );
% wGuess = rand( fDim, wDim );

initParams = trueParams;
initParams.v = vGuess;
initParams.w = wGuess;
initParams.vPenalty = vPenalty;
initParams.wPenalty = wPenalty;

% KF Params
S0 = 1E-3*eye(xDim);

% Plot params
linewidth = 2;

damping = 1E-6*eye(zDim);
trainDamping = repmat( damping, [1,1,trainT] );
testDamping = repmat( damping, [1,1,testT] );

%% Generate train and test trajectories
trainTrajNoise = mvnrnd( zeros(1,xDim), Q, trainT)'; % Process noise
testTrajNoise = mvnrnd( zeros(1,xDim), Q, testT)';

trainTrueStates = zeros(xDim,trainT);
trainTrueStates(:,1) = trainX0;
testTrueStates = zeros(xDim,testT);
testTrueStates(:,1) = testX0;

for i = 2:trainT
    trainTrueStates(:,i) = A*trainTrueStates(:,i-1) + trainTrajNoise(:,i);
end
for i = 2:testT
    testTrueStates(:,i) = A*testTrueStates(:,i-1) + testTrajNoise(:,i);
end

trainTimes = dt*(1:trainT);
testTimes = dt*(1:testT);

% Generate true features
trainTrueFeatures = featureFunc(trainTrueStates);
testTrueFeatures = featureFunc(testTrueStates);

% Generate observation means
trainTrueObservations = C*trainTrueStates;
testTrueObservations = C*testTrueStates;

% Generate observation noise info matrices
trainTrueCov = ModifiedCholeskyRegression( trainTrueFeatures, trueParams, true );
testTrueCov = ModifiedCholeskyRegression( testTrueFeatures, trueParams, true );

% trainTrueCov = zeros(zDim, zDim, trainT);
% for i = 1:trainT
%     trainTrueCov(:,:,i) = 5*abs(trainTrueStates(2,i))*eye(2) + 1E-3*eye(2);
% end
% 
% testTrueCov = zeros(zDim, zDim, testT);
% for i = 1:testT
%     testTrueCov(:,:,i) = 5*abs(testTrueStates(2,i))*eye(2) + 1E-3*eye(2);
% end

% Sample observation noise
trainTrueObsNoise = mvnrnd( zeros(zDim,trainT)', trainTrueCov )';
testTrueObsNoise = mvnrnd( zeros(zDim,testT)', testTrueCov )';

trainObsNoised = trainTrueObservations + trainTrueObsNoise;
testObsNoised = testTrueObservations + testTrueObsNoise;

%% Plot all the things
% Plot true trajectory
trajectoryFig = figure;
trajectoryAxes = axes( 'parent', trajectoryFig );
plot( trajectoryAxes, testTimes, testTrueStates(1,:), 'b-' );
hold( trajectoryAxes, 'on' );
title( trajectoryAxes, 'Trajectory' );

% Plot trajectory errors
errFig = figure;
posErrAxes = subplot(3,1,1, 'parent', errFig);
hold( posErrAxes, 'on' );
title( posErrAxes, 'Position Errors' );
ylabel('Position, x (m)')

velErrAxes = subplot(3,1,2, 'parent', errFig );
hold( velErrAxes, 'on' );
title( velErrAxes, 'Velocity Errors' );
ylabel('Velocity, v (m/s)')

accErrAxes = subplot(3,1,3, 'parent', errFig );
hold( accErrAxes, 'on' );
title( accErrAxes, 'Acceleration Errors' );
ylabel('Acceleration, a (m/s^{2})');
xlabel('Time, t (s)');

% Plot position measurements
posFig = figure;
posAxes = axes( 'parent', posFig );
hold( posAxes, 'on' );
plot( posAxes, testTimes, testTrueObservations(1,:), 'b-' );
plot( posAxes, testTimes, testTrueObservations(1,:) +  testTrueObsNoise(1,:), 'b.' );
plot( posAxes, testTimes, testTrueObservations(1,:)' + sqrt(squeeze(testTrueCov(1,1,:))), 'g-' );
plot( posAxes, testTimes, testTrueObservations(1,:)' - sqrt(squeeze(testTrueCov(1,1,:))), 'g-' );
title( posAxes, 'Position observations' );

% Plot velocity measurements
velFig = figure;
velAxes = axes( 'parent', velFig );
hold( velAxes, 'on' );
plot( velAxes, testTimes, testTrueObservations(2,:), 'b-' );
plot( velAxes, testTimes, testTrueObservations(2,:) + testTrueObsNoise(2,:), 'b.' );
plot( velAxes, testTimes, testTrueObservations(2,:)' + sqrt(squeeze(testTrueCov(2,2,:))), 'g-' );
plot( velAxes, testTimes, testTrueObservations(2,:)' - sqrt(squeeze(testTrueCov(2,2,:))), 'g-' );
title( velAxes, 'Velocity observations' );

% Plot velocity measurements
likeFig = figure;
likeAxes = axes( 'parent', likeFig );
hold( likeAxes, 'on' );
title( likeAxes, 'Trajectory log likelihoods' );

%% MLE Baseline
% Covariance that maximizes observation likelihood is sample cov
mleR = cov(trainTrueObsNoise');

mleTestStates = zeros(xDim,testT);
mleCovs = zeros(xDim,xDim,testT);

mleKF = KalmanFilter(Aest, [], Qest, C, []);
mleKF.Initialize(testX0, S0);
for i = 1:testT
    mleKF.Update( testObsNoised(:,i), mleR );
    mleTestStates(:,i) = mleKF.x;
    mleCovs(:,:,i) = mleKF.S;
    
    mleKF.Predict();
end

plot( trajectoryAxes, testTimes, mleTestStates(1,:), 'r-' );

mleLikelihood = GaussianLogLikelihood( testTrueStates - mleTestStates, mleCovs, 'covariance' );
mleTrajLikelihood = mean(mleLikelihood);
plot( likeAxes, testTimes, mleLikelihood, 'r-' );
fprintf( 'MLE trajectory test likelihood: %f\n', mleTrajLikelihood );

mleTestDataLikelihood = mean( GaussianLogLikelihood( testTrueObsNoise, repmat(mleR, [1,1,testT]), 'covariance' ) );
fprintf( 'MLE noise test likelihood: %f\n', mleTestDataLikelihood );

errMLE = mleTestStates - testTrueStates;
errMLEMean = mean( abs(errMLE), 2 );
plot( posErrAxes, testTimes, abs(errMLE(1,:)), 'r-', 'linewidth', linewidth );
plot( velErrAxes, testTimes, abs(errMLE(2,:)), 'r-', 'linewidth', linewidth );
plot( accErrAxes, testTimes, abs(errMLE(3,:)), 'r-', 'linewidth', linewidth );

% Show on observation plots
plot( posAxes, testTimes, testTrueObservations(1,:) + sqrt(mleR(1,1)), 'r-' );
plot( posAxes, testTimes, testTrueObservations(1,:) - sqrt(mleR(1,1)), 'r-' );

plot( velAxes, testTimes, testTrueObservations(2,:) + sqrt(mleR(2,2)), 'r-' );
plot( velAxes, testTimes, testTrueObservations(2,:) - sqrt(mleR(2,2)), 'r-' );

%% Run KF with true covariances
tcovTestStates = zeros(xDim,testT);
tcovTestCovs = zeros(xDim,xDim,testT);

tcovTrainStates = zeros(xDim,trainT);
tcovTrainCovs = zeros(xDim,xDim,trainT);

tcovTestInnovations = zeros(zDim,testT);
tcovTestPrioriCovs = zeros(zDim,zDim,testT);
tcovTestFeatures = zeros(fDim,testT);

tcovTrainInnovations = zeros(zDim,trainT);
tcovTrainPrioriCovs = zeros(zDim,zDim,trainT);
tcovTrainFeatures = zeros(fDim,trainT);

tcovKF = KalmanFilter(Aest, [], Qest, C, []);

tcovKF.Initialize(trainX0, S0);
for i = 1:trainT
    tcovKF.Update( trainObsNoised(:,i), trainTrueCov(:,:,i) );
    tcovTrainStates(:,i) = tcovKF.x;
    tcovTrainCovs(:,:,i) = tcovKF.S;
    tcovTrainFeatures(:,i) = featureFunc( tcovKF.x );
    
    tcovKF.Predict();
    tcovTrainInnovations(:,i) = trainObsNoised(:,i) - C*tcovKF.x;
    tcovTrainPrioriCovs(:,:,i) = C*tcovKF.S*C';
end

tcovKF.Initialize(testX0, S0);
for i = 1:testT
    tcovKF.Update( testObsNoised(:,i), testTrueCov(:,:,i) );
    tcovTestStates(:,i) = tcovKF.x;
    tcovTestCovs(:,:,i) = tcovKF.S;
    tcovTestFeatures(:,i) = featureFunc( tcovKF.x );
    
    tcovKF.Predict();
    tcovTestInnovations(:,i) = testObsNoised(:,i) - C*tcovKF.x;
    tcovTestPrioriCovs(:,:,i) = C*tcovKF.S*C';
end

plot( trajectoryAxes, testTimes, tcovTestStates(1,:), 'g-' );

errTrueF = tcovTestStates - testTrueStates;
errTrueFMean = mean( abs(errTrueF), 2 );

tcovTestLikelihood = GaussianLogLikelihood( testTrueStates - tcovTestStates, tcovTestCovs, 'covariance' );
tcovTestTrajLikelihood = mean(tcovTestLikelihood);
plot( likeAxes, testTimes, tcovTestLikelihood, 'g-' );
fprintf( 'True trajectory test likelihood: %f\n', tcovTestTrajLikelihood );

tcovTestDataLikelihood = mean( GaussianLogLikelihood( testTrueObsNoise, testTrueCov, 'covariance' ) );
fprintf( 'True noise test likelihood: %f\n', tcovTestDataLikelihood );

plot( posErrAxes, testTimes, abs(errTrueF(1,:)), 'g-', 'linewidth', linewidth );
plot( velErrAxes, testTimes, abs(errTrueF(2,:)), 'g-', 'linewidth', linewidth );
plot( accErrAxes, testTimes, abs(errTrueF(3,:)), 'g-', 'linewidth', linewidth );

%% Test of positive basis model
basisDim = 5;
pbParams.w = zeros(fDim, basisDim);
pbParams.basis = zeros(zDim,zDim,basisDim);
pbParams.wPenalty = 1E-2;
for i = 1:basisDim-1
   A = rand(zDim,1) - 0.5;
   pd = A*A';
   pd = pd/trace(pd); % Uniform trace-power
   pbParams.basis(:,:,i) = pd;
end

pbParams.basis(:,:,end) = eye(zDim)/zDim;

pbParams.modelMode = 'covariance';
pbParams.dataMode = 'innovation';

pbObjectiveFunction = @(p) PositiveBasisObjectiveFunction( p, pbParams, ...
    tcovTrainInnovations, tcovTrainFeatures, tcovTrainPrioriCovs + trainDamping );

options = optimoptions( 'fminunc', 'algorithm', 'quasi-newton', 'GradObj','on', ...
        'diagnostics', 'off', 'derivativecheck', 'off', ...
        'plotfcns', {@optimplotx, @optimplotfval}, ...
        'tolfun', 0 );
pbParamVec = fminunc( pbObjectiveFunction, zeros(1,fDim*basisDim), options );
pbParams.w = reshape( pbParamVec, size(pbParams.w) );

% pbParams = PositiveBasisAscent( trainTrueObsNoise, trainTrueFeatures, ...
%     trainDamping, pbParams, ascentParams );

pbR = PositiveBasisRegression( tcovTestFeatures, pbParams ) + testDamping;

pbTestStates = zeros(xDim,testT);
pbCovs = zeros(xDim,xDim,testT);
pbTestStates(:,1) = testX0;

pbKF = KalmanFilter(Aest, [], Qest, C, []);
pbKF.Initialize(testX0, S0);
for i = 1:testT
    fi = featureFunc( pbKF.x );
    pbRi = PositiveBasisRegression( fi, pbParams );
    pbKF.Update( testObsNoised(:,i), pbR(:,:,i) );
    pbTestStates(:,i) = pbKF.x;
    pbCovs(:,:,i) = pbKF.S;
    
    pbKF.Predict();
end

plot( trajectoryAxes, testTimes, pbTestStates(1,:), 'k' );

pbTestLikelihood = GaussianLogLikelihood( testTrueStates - pbTestStates, pbCovs, 'covariance' );
pbTestTrajLikelihood = mean(pbTestLikelihood);
plot( likeAxes, testTimes, pbTestLikelihood, 'k-' );
fprintf( 'Positive basis trajectory likelihood: %f\n', pbTestTrajLikelihood );

pbTestDataLikelihood = mean( GaussianLogLikelihood( testTrueObsNoise, pbR, 'covariance' ) );
fprintf( 'Positve basis test noise likelihood: %f\n', pbTestDataLikelihood );

pbErr = pbTestStates - testTrueStates;
pbErrMean = mean( abs(pbErr), 2 );
plot( posErrAxes, testTimes, abs(pbErr(1,:)), 'k-' );
plot( velErrAxes, testTimes, abs(pbErr(2,:)), 'k-' );
plot( accErrAxes, testTimes, abs(pbErr(3,:)), 'k-' );

% Show GLI regressed variance on plot
plot( posAxes, testTimes, testTrueObservations(1,:)' + sqrt(squeeze(pbR(1,1,:))), 'k-' );
plot( posAxes, testTimes, testTrueObservations(1,:)' - sqrt(squeeze(pbR(1,1,:))), 'k-' );

plot( velAxes, testTimes, testTrueObservations(2,:)' + sqrt(squeeze(pbR(2,2,:))), 'k-' );
plot( velAxes, testTimes, testTrueObservations(2,:)' - sqrt(squeeze(pbR(2,2,:))), 'k-' );


%% Run KF with regressed innovation covariances
ricParams.modelMode = 'covariance';
ricParams.dataMode = 'innovation';
ricParams.w = zeros(fDim,wDim);
ricParams.v = zeros(fDim,vDim);
ricParams.vPenalty = 0;%1E-3;
ricParams.wPenalty = 0;

ricObjectiveFunction = @(p) ModifiedCholeskyObjectiveFunction( p, ricParams, ...
    tcovTrainInnovations, tcovTrainFeatures, tcovTrainPrioriCovs + trainDamping );

options = optimoptions( 'fminunc', 'algorithm', 'quasi-newton', 'GradObj','on', ...
        'diagnostics', 'off', 'derivativecheck', 'off', ...
        'plotfcns', {@optimplotx, @optimplotfval}, ...
        'tolfun', 0 );
ricInitVec = [ricParams.v(:); ricParams.w(:)];
ricParamVec = fminunc( ricObjectiveFunction, ricInitVec, options );

ricParams.v = reshape( ricParamVec(1:fDim*vDim), fDim, vDim );
ricParams.w = reshape( ricParamVec(fDim*vDim + (1:fDim*wDim)), fDim, wDim );


% Custom gradient ascent
% ricParams = GaussianLikelihoodAscent( tcovTrainInnovations, tcovTrainFeatures, ...
%     tcovTrainPrioriCovs + trainDamping, initParams, ascentParams );

ricR = ModifiedCholeskyRegression( tcovTestFeatures, ricParams, true ) + testDamping;

ricTestStates = zeros(xDim,testT);
ricCovs = zeros(xDim,xDim,testT);
ricTestStates(:,1) = testX0;

ricKF = KalmanFilter(Aest, [], Qest, C, []);
ricKF.Initialize(testX0, S0);
for i = 1:testT
    fi = featureFunc( ricKF.x );
    ricRi = ModifiedCholeskyRegression( fi, ricParams, true );
    ricKF.Update( testObsNoised(:,i), ricR(:,:,i) );
    ricTestStates(:,i) = ricKF.x;
    ricCovs(:,:,i) = ricKF.S;
    
    ricKF.Predict();
end

plot( trajectoryAxes, testTimes, ricTestStates(1,:), 'm' );

ricTestLikelihood = GaussianLogLikelihood( testTrueStates - ricTestStates, ricCovs, 'covariance' );
ricTestTrajLikelihood = mean(ricTestLikelihood);
plot( likeAxes, testTimes, ricTestLikelihood, 'm-' );
fprintf( 'Innovation covariance trajectory likelihood: %f\n', ricTestTrajLikelihood );

ricTestDataLikelihood = mean( GaussianLogLikelihood( testTrueObsNoise, ricR, 'covariance' ) );
fprintf( 'Innovation covariance noise likelihood: %f\n', ricTestDataLikelihood );

ricErr = ricTestStates - testTrueStates;
ricErrMean = mean( abs(ricErr), 2 );
plot( posErrAxes, testTimes, abs(ricErr(1,:)), 'm-' );
plot( velErrAxes, testTimes, abs(ricErr(2,:)), 'm-' );
plot( accErrAxes, testTimes, abs(ricErr(3,:)), 'm-' );

% Show GLI regressed variance on plot
plot( posAxes, testTimes, testTrueObservations(1,:)' + sqrt(squeeze(ricR(1,1,:))), 'm-' );
plot( posAxes, testTimes, testTrueObservations(1,:)' - sqrt(squeeze(ricR(1,1,:))), 'm-' );

plot( velAxes, testTimes, testTrueObservations(2,:)' + sqrt(squeeze(ricR(2,2,:))), 'm-' );
plot( velAxes, testTimes, testTrueObservations(2,:)' - sqrt(squeeze(ricR(2,2,:))), 'm-' );

%% Run KF with regressed innovation covariances using full L model
% ficParams.modelMode = 'fullcovariance';
% ficParams.dataMode = 'innovation';
% ficParams.w = zeros(fDim,wDim);
% fvDim = wDim*wDim;
% ficParams.v = zeros(fDim,fvDim);
% ficParams.vPenalty = 1E-1;
% ficParams.wPenalty = 0;
% 
% ficObjectiveFunction = @(p) ModifiedCholeskyObjectiveFunction( p, ficParams, ...
%     tcovTrainInnovations, tcovTrainFeatures, tcovTrainPrioriCovs + trainDamping );
% 
% options = optimoptions( 'fminunc', 'algorithm', 'quasi-newton', 'GradObj','on', ...
%         'diagnostics', 'off', 'derivativecheck', 'off', ...
%         'plotfcns', {@optimplotx, @optimplotfval}, ...
%         'tolfun', 0 );
% ficInitVec = [ficParams.v(:); ficParams.w(:)];
% ficParamVec = fminunc( ficObjectiveFunction, ficInitVec, options );
% 
% ficParams.v = reshape( ficParamVec(1:fDim*fvDim), fDim, fvDim );
% ficParams.w = reshape( ficParamVec(fDim*fvDim + (1:fDim*wDim)), fDim, wDim );
% 
% ficR = ModifiedCholeskyRegression( tcovTestFeatures, ficParams, true ) + testDamping;
% 
% ficTestStates = zeros(xDim,testT);
% ficCovs = zeros(xDim,xDim,testT);
% ficTestStates(:,1) = testX0;
% 
% ficKF = KalmanFilter(Aest, [], Qest, C, []);
% ficKF.Initialize(testX0, S0);
% for i = 1:testT
%     fi = featureFunc( ficKF.x );
%     ficRi = ModifiedCholeskyRegression( fi, ficParams, true );
%     ficKF.Update( testObsNoised(:,i), ficR(:,:,i) );
%     ficTestStates(:,i) = ficKF.x;
%     ficCovs(:,:,i) = ficKF.S;
%     
%     ficKF.Predict();
% end
% 
% plot( trajectoryAxes, testTimes, ficTestStates(1,:), 'c' );
% 
% ficTestLikelihood = GaussianLogLikelihood( testTrueStates - ficTestStates, ficCovs, 'covariance' );
% ficTestTrajLikelihood = mean(ficTestLikelihood);
% plot( likeAxes, testTimes, ficTestLikelihood, 'c-' );
% fprintf( 'Innovation full covariance trajectory likelihood: %f\n', ficTestTrajLikelihood );
% 
% ficTestDataLikelihood = mean( GaussianLogLikelihood( testTrueObsNoise, ficR, 'covariance' ) );
% fprintf( 'Innovation full covariance noise likelihood: %f\n', ficTestDataLikelihood );
% 
% ficErr = ficTestStates - testTrueStates;
% ficErrMean = mean( abs(ficErr), 2 );
% plot( posErrAxes, testTimes, abs(ficErr(1,:)), 'c-' );
% plot( velErrAxes, testTimes, abs(ficErr(2,:)), 'c-' );
% plot( accErrAxes, testTimes, abs(ficErr(3,:)), 'c-' );
% 
% % Show GLI regressed variance on plot
% plot( posAxes, testTimes, testTrueObservations(1,:)' + sqrt(squeeze(ficR(1,1,:))), 'c-' );
% plot( posAxes, testTimes, testTrueObservations(1,:)' - sqrt(squeeze(ficR(1,1,:))), 'c-' );
% 
% plot( velAxes, testTimes, testTrueObservations(2,:)' + sqrt(squeeze(ficR(2,2,:))), 'c-' );
% plot( velAxes, testTimes, testTrueObservations(2,:)' - sqrt(squeeze(ficR(2,2,:))), 'c-' );

%% Plot matrix elements

covFig = figure;
covAxes = axes( 'parent', covFig );
hold( covAxes, 'on' );
title( covAxes, 'Covariance matrix elements' );

plot( covAxes, squeeze(ricR(1,1,:)), 'm-' )
plot( covAxes, squeeze(ricR(2,2,:)), 'm:' )
plot( covAxes, squeeze(ricR(1,2,:)), 'k--' )

% plot( covAxes, squeeze(ficR(1,1,:)), 'c-' )
% plot( covAxes, squeeze(ficR(2,2,:)), 'c:' )
% plot( covAxes, squeeze(ficR(1,2,:)), 'c--' )

%% Run KF with regressed error covariances
% recParams = ErrorCovarianceGradientAscent( trainTrueObsNoise, trainTrueFeatures, ...
%     initParams, ascentParams );
% recR = ModifiedCholeskyRegression( testTrueFeatures, recParams, true );
% 
% recTestStates = zeros(xDim,testT);
% recCovs = zeros(xDim,xDim,testT);
% 
% recKF = KalmanFilter(Aest, [], Qest, C, []);
% recKF.Initialize(x0, S0);
% for i = 1:testT
%     recKF.Update( testObsNoised(:,i), recR(:,:,i) );
%     recTestStates(:,i) = recKF.x;
%     recCovs(:,:,i) = recKF.S;
%     
%     recKF.Predict();
% end
% 
% plot( trajectoryAxes, testTimes, recTestStates(1,:), 'c-' );
% 
% recTestLikelihood = GaussianLogLikelihood( testTrueStates - recTestStates, recCovs, 'cov' );
% recTestTrajLikelihood = sum(recTestLikelihood);
% plot( likeAxes, testTimes, recTestLikelihood, 'c-' );
% fprintf( 'Error covariance trajectory likelihood: %f\n', recTestTrajLikelihood );
% 
% recErr = recTestStates - testTrueStates;
% recErrMean = mean( abs(recErr), 2 );
% plot( posErrAxes, testTimes, abs(recErr(1,:)), 'c-' );
% plot( velErrAxes, testTimes, abs(recErr(2,:)), 'c-' );
% plot( accErrAxes, testTimes, abs(recErr(3,:)), 'c-' );
% 
% % Show GLI regressed variance on plot
% plot( posAxes, testTimes, testTrueObservations(1,:)' + sqrt(squeeze(recR(1,1,:))), 'c-' );
% plot( posAxes, testTimes, testTrueObservations(1,:)' - sqrt(squeeze(recR(1,1,:))), 'c-' );
% 
% plot( velAxes, testTimes, testTrueObservations(2,:)' + sqrt(squeeze(recR(2,2,:))), 'c-' );
% plot( velAxes, testTimes, testTrueObservations(2,:)' - sqrt(squeeze(recR(2,2,:))), 'c-' );

%% Run KF with regressed error informations
% reiParams = ErrorInformationGradientAscent( trainTrueObsNoise, trainTrueFeatures, ...
%     initParams, ascentParams );
% reiInfo = ModifiedCholeskyRegression( testTrueFeatures, reiParams, true );
% reiR = zeros(zDim,zDim,testT);
% for i = 1:testT
%    reiR(:,:,i) = pinv(reiInfo(:,:,i)); 
% end
% 
% reiTestStates = zeros(xDim,testT);
% reiCovs = zeros(xDim,xDim,testT);
% 
% reiIF = InformationFilter(Aest, [], Qest, C, []);
% reiIF.Initialize(x0, S0);
% for i = 1:testT
%     reiIF.Update( testObsNoised(:,i), reiInfo(:,:,i) );
%     [reiTestStates(:,i), reiCovs(:,:,i)] = reiIF.GetState();
%     
%     reiIF.Predict();
%     
% end
% 
% plot( trajectoryAxes, testTimes, reiTestStates(1,:), 'm' );
% 
% reiTestLikelihood = GaussianLogLikelihood( testTrueStates - reiTestStates, reiCovs, 'cov' );
% reiTestTrajLikelihood = sum(reiTestLikelihood);
% plot( likeAxes, testTimes, reiTestLikelihood, 'm-' );
% fprintf( 'Error information trajectory likelihood: %f\n', reiTestTrajLikelihood );
% 
% reiErr = reiTestStates - testTrueStates;
% reiErrMean = mean( abs(reiErr), 2 );
% plot( posErrAxes, testTimes, abs(reiErr(1,:)), 'm-' );
% plot( velErrAxes, testTimes, abs(reiErr(2,:)), 'm-' );
% plot( accErrAxes, testTimes, abs(reiErr(3,:)), 'm-' );
% 
% % Show GLI regressed variance on plot
% plot( posAxes, testTimes, testTrueObservations(1,:)' + sqrt(squeeze(reiR(1,1,:))), 'm-' );
% plot( posAxes, testTimes, testTrueObservations(1,:)' - sqrt(squeeze(reiR(1,1,:))), 'm-' );
% 
% plot( velAxes, testTimes, testTrueObservations(2,:)' + sqrt(squeeze(reiR(2,2,:))), 'm-' );
% plot( velAxes, testTimes, testTrueObservations(2,:)' - sqrt(squeeze(reiR(2,2,:))), 'm-' );

% %% Test online MLE
% 
% omleBuff = [];
% omleStates = zeros(xDim,T);
% 
% omleCovs = zeros(zDim,zDim,T);
% 
% omleStates(:,1) = x0;
% 
% omleKF = KalmanFilter(Aest, [], Q, C, []);
% omleKF.Initialize(x0, S0);
% 
% % Give a few time steps to fill the MLE buffer a bit
% burnIn = 10;
% for i = 1:burnIn
%     omleBuff = [omleBuff, trueObsNoise(:,i)];
%     omleStates(:,i) = omleKF.x;
%     omleKF.Predict();
% end
%     
% for i = burnIn+1:T
%     omleCovs(:,:,i) = cov( omleBuff' );
%     omleBuff = [omleBuff, trueObsNoise(:,i)];
%     omleKF.Update( obsNoised(:,i), omleCovs(:,:,i) );
%     omleStates(:,i) = omleKF.x;
%     omleKF.Predict();
% end
% 
% plot( trajectoryAxes, times, omleStates(1,:), 'r--' );
% 
% omleErr = omleStates - trueStates;
% plot( posErrAxes, times, abs(omleErr(1,:)), 'r--' );
% plot( velErrAxes, times, abs(omleErr(2,:)), 'r--' );
% plot( accErrAxes, times, abs(omleErr(3,:)), 'r--' );
% 
% % Show on observation plots
% plot( posAxes, times, trueObservations(1,:)' + sqrt(squeeze(omleCovs(1,1,:))), 'r--' );
% plot( posAxes, times, trueObservations(1,:)' - sqrt(squeeze(omleCovs(1,1,:))), 'r--' );
% 
% plot( velAxes, times, trueObservations(2,:)' + sqrt(squeeze(omleCovs(2,2,:))), 'r--' );
% plot( velAxes, times, trueObservations(2,:)' - sqrt(squeeze(omleCovs(2,2,:))), 'r--' );
% 
% %% Test online GLI
% onlineParams = initParams;
% onlineParams.vPenalty = 1E-2;
% onlineParams.wPenalty = 1E-2;
% onlineAscentParams = ascentParams;
% onlineAscentParams.maximumIterations = 1;
% onlineAscentParams.showPlots = false;
% onlineAscentParams.maximumBacktracks = 0;
% 
% ogliStates = zeros(xDim,T);
% ogliCovs = zeros(zDim,zDim,T);
% 
% ogliState(:,1) = x0;
% 
% ogliKF = KalmanFilter(Aest, [], Q, C, []);
% ogliKF.Initialize(x0, S0);
% 
% for i = 1:T
%     ogliCovs(:,:,i) = inv(ModifiedCholeskyRegression(trueFeatures(:,i), onlineParams, true));
%     
%     ogliKF.Update( obsNoised(:,i), ogliCovs(:,:,i) );
%     ogliStates(:,i) = ogliKF.x;
%     ogliKF.Predict();
%     
%     % Update step
%     onlineAscentParams.stepSize = 1E-1/sqrt(i);
%     onlineParams = GLILikelihoodGradientAscent( trueObsNoise(:,i), trueFeatures(:,i), ...
%         onlineParams, onlineAscentParams );
% end
% 
% 
% plot( trajectoryAxes, times, ogliStates(1,:), 'm--' );
% 
% ogliErr = ogliStates - trueStates;
% plot( posErrAxes, times, abs(ogliErr(1,:)), 'm--' );
% plot( velErrAxes, times, abs(ogliErr(2,:)), 'm--' );
% plot( accErrAxes, times, abs(ogliErr(3,:)), 'm--' );
% 
% % Show on observation plots
% plot( posAxes, times, trueObservations(1,:)' + sqrt(squeeze(ogliCovs(1,1,:))), 'm--' );
% plot( posAxes, times, trueObservations(1,:)' - sqrt(squeeze(ogliCovs(1,1,:))), 'm--' );
% 
% plot( velAxes, times, trueObservations(2,:)' + sqrt(squeeze(ogliCovs(2,2,:))), 'm--' );
% plot( velAxes, times, trueObservations(2,:)' - sqrt(squeeze(ogliCovs(2,2,:))), 'm--' );