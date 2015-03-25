% Simulate a 2D car moving around getting range and bearing measurements
% to a landmark with range-dependent uncertainty

%% Parameters

% Trajectory parameters
% All positions are [x;y;theta]
cartStartPos = [0;0;pi/2];
beaconPos = [1, 0, 0;
    -1, 0, 0]';
T = 1000;    % Time steps
dt = 0.3;
times = dt*(1:T);

radius = 1;
speed = 0.1;

steerAmplitude = 0.5;
steerFrequency = 0;
steerOffset = 0;

controlFunc = @(x,b,t) [ speed; -speed/(radius) + t*dt*steerOffset ]; %+ ...
%[ zeros(1, numel(t)); steerAmplitude*cos(t*dt*steerFrequency) ];
rotmat = @(th) [ cos(th), -sin(th);
    sin(th), cos(th) ];
drotmat = @(th) [ -sin(th), -cos(th);
    cos(th), -sin(th) ];
transitionFunc = @(x,u) x + dt*[ rotmat(x(3))*[u(1);0]; u(2) ];

% GLI Parameters
xDim = 3; %[x;y;theta;xdot;ydot;thetadot]
zDim = 4;
uDim = 2;
fDim = 9;

wDim = 4;
vDim = zDim*(zDim-1)/2;

trueParams.v = zeros( fDim, vDim);
% range1, bearing1, range2, bearing2
trueParams.w = [ 0, -1, 0, 0;     % Range 1
    -5, -1, 0, 0;   % Range 1 squared
    0, 0, 0, 0;       % Bearing 1 magnitude
    0, 0, 0, 0;       % Bearing 1 squared
    0, 0, 0, -1;     % Range 2
    0, 0, -5, -1;   % Range 2 squared
    0, 0, 0, 0;       % Bearing 2 magnitude
    0, 0, 0, 0;       % Bearing 2 squared
    7, 7, 7, 7 ];     % Constant offset
trueParams.vPenalty = 1E-3;
trueParams.wPenalty = 0;

initParams = trueParams;
initParams.v = zeros( fDim, vDim );
initParams.w = zeros( fDim, wDim );

ascentParams.stepSize = 1E-1;
ascentParams.backtrackAlpha = 0.2;
ascentParams.backtrackBeta = 0.8;
ascentParams.gradientTolerance = 1E-3;
ascentParams.maximumIterations = 1000;
ascentParams.maximumBacktracks = 10;
ascentParams.showPlots = true;

% The mean range-bearing observation function (no noise)
obsFunc = @(cPos, bPos) [ norm( cPos(1:2,:) - bPos(1:2,:) );
    wrapToPi(atan2(bPos(2,:) - cPos(2,:), bPos(1,:) - cPos(1,:)) - cPos(3,:)) ];

% Observation features are range, range squared, bearing magnitude, bearing
% squared, and range-bearing magnitude cross term.
obsFeatFunc = @(obs) [ log(obs(1,:));
    log(obs(1,:).^2);
    sqrt(abs(obs(2,:)));
    abs(obs(2,:));
    log(obs(3,:));
    log(obs(3,:).^2);
    sqrt(abs(obs(4,:)));
    abs(obs(4,:));
    ones(1,size(obs,2)) ];

% EKF function handles
ekfTransFunc = @(x,u) transitionFunc(x,u);
ekfTransJac = @(x,u) eye(xDim) + [ zeros(xDim,2), [drotmat(x(3))*[u(1);0]; 0] ];

ekfObsFeatFunc = @(z) obsFeatFunc(z);
ekfObsFunc = @(x) [obsFunc(x, beaconPos(:,1)); obsFunc(x, beaconPos(:,2) )];
ekfObsJac = @(x) [RangeBearingJacobian(x, beaconPos(:,1)), RangeBearingJacobian(x, beaconPos(:,2))];
ekfObsDiff = @(z,w) [z(1,:) - w(1,:); wrapToPi( z(2,:) - w(2,:) );
    z(3,:) - w(3,:); wrapToPi( z(4,:) - w(4,:) )];

Qtrue = diag([1E-6,1E-6,1E-4]);
Q = 1*Qtrue;
S0 = (2E-2*eye(xDim)).^2;

%% Run the cart trajectory

trueStates = zeros(xDim,T);
trueStates(:,1) = cartStartPos;

trueControls = zeros(uDim,T);
trueInfo = zeros(zDim,zDim,T);
trueCov = zeros(zDim,zDim,T);
trueTransNoise = zeros(xDim,T);
trueObsScale = 1.5*rand(2,T) + 0.5;
trueFeatures = zeros(fDim,T);
trueObservations = zeros(zDim,T);
noisedObservations = zeros(zDim,T);

for t = 1:T-1
    trueControls(:,t) = controlFunc(trueStates(:,t), beaconPos,t);
    trueTransNoise(:,t) = mvnrnd( zeros(1,xDim), Qtrue )';
    trueStates(:,t+1) = transitionFunc(trueStates(:,t), trueControls(:,t)) + trueTransNoise(:,t);
end

for t = 1:T
    trueObservations(:,t) = ekfObsFunc( trueStates(:,t) );
    trueFeatures(:,t) = obsFeatFunc( trueObservations(:,t) );
    %     trueInfo(:,:,t) = ModifiedCholeskyRegression( trueFeatures(:,t), trueParams, true );
    %     trueCov(:,:,t) = inv(trueInfo(:,:,t));
end

% Scale the range measurements
noisedObservations(1,:) = trueObsScale(1,:).*trueObservations(1,:);
noisedObservations(3,:) = trueObsScale(2,:).*trueObservations(3,:);

% Gaussian corrupt the bearing measurements
% trueObsNoise(:,t) = mvnrnd( zeros(1,zDim), trueCov(:,:,t) )';
noisedObservations(2,:) = wrapToPi( trueObservations(2,:) + mvnrnd( 0, 1E-3, T )' );
noisedObservations(4,:) = wrapToPi( trueObservations(4,:) + mvnrnd( 0, 1E-3, T )' );

trueObsNoise = noisedObservations - trueObservations;
trueObsNoise([2,4],:) = wrapToPi( trueObsNoise([2,4],:) );
%% Plots

trajectoryFig = figure;
trajectoryAxes = axes( 'parent', trajectoryFig );
xlabel('x');
ylabel('y');
axis( trajectoryAxes, 'equal' );
hold( trajectoryAxes, 'on' );
title('Trajectory');
plot( trajectoryAxes, trueStates(1,:), trueStates(2,:), 'b-' );
plot( trajectoryAxes, beaconPos(1,:), beaconPos(2,:), 'bo', 'linewidth', 2 );

rangeFig = figure;
rangeAxes = axes( 'parent', rangeFig );
xlabel('t');
ylabel('z');
title('Range observations');
hold( rangeAxes, 'on' );
plot( rangeAxes, times, trueObservations(1,:), 'b-' );
plot( rangeAxes, times, trueObservations(1,:) + trueObsNoise(1,:), 'b.');

bearingFig = figure;
bearingAxes = axes( 'parent', bearingFig );
xlabel('t');
ylabel('z');
title('Bearing observations');
hold( bearingAxes, 'on' );
plot( bearingAxes, times, trueObservations(2,:), 'b-' );
plot( bearingAxes, times, trueObservations(2,:) + trueObsNoise(2,:), 'b.');

range2Fig = figure;
range2Axes = axes( 'parent', range2Fig );
xlabel('t');
ylabel('z');
title('Range 2 observations');
hold( range2Axes, 'on' );
plot( range2Axes, times, trueObservations(3,:), 'b-' );
plot( range2Axes, times, trueObservations(3,:) + trueObsNoise(3,:), 'b.');

bearing2Fig = figure;
bearing2Axes = axes( 'parent', bearing2Fig );
xlabel('t');
ylabel('z');
title('Bearing 2 observations');
hold( bearing2Axes, 'on' );
plot( bearing2Axes, times, trueObservations(4,:), 'b-' );
plot( bearing2Axes, times, trueObservations(4,:) + trueObsNoise(4,:), 'b.');

errFig = figure;
posErrAxes = axes( 'parent', errFig );
title('Position errors');
hold( posErrAxes, 'on' );

%% Run an EKF with the batch MLE noise covariance
mleCov = cov(trueObsNoise');

mleEKF = ExtendedKalmanFilter( ekfTransFunc, ekfTransJac, ekfObsFunc, ekfObsJac, ekfObsDiff );
mleEKF.Initialize( trueStates(:,1), S0 );

mleStates = zeros(xDim,T);
mleStates(:,1) = mleEKF.x;
for t = 1:T
    mleEKF.Update(noisedObservations(:,t), mleCov);
    mleStates(:,t) = mleEKF.x;
    mleEKF.Predict(trueControls(:,t), Q);
end

mleErrs = mleStates - trueStates;
mlePosErrs = smooth( vnorm( mleErrs(1:2,:) ), 20 );

plot( trajectoryAxes, mleStates(1,:), mleStates(2,:), 'r-' );

plot( rangeAxes, times, trueObservations(1,:) + sqrt(mleCov(1,1))', 'r-' );
plot( rangeAxes, times, trueObservations(1,:) - sqrt(mleCov(1,1))', 'r-' );
plot( bearingAxes, times, trueObservations(2,:) + sqrt(mleCov(2,2))', 'r-' );
plot( bearingAxes, times, trueObservations(2,:) - sqrt(mleCov(2,2))', 'r-' );

plot( range2Axes, times, trueObservations(3,:) + sqrt(mleCov(3,3))', 'r-' );
plot( range2Axes, times, trueObservations(3,:) - sqrt(mleCov(3,3))', 'r-' );
plot( bearing2Axes, times, trueObservations(4,:) + sqrt(mleCov(4,4))', 'r-' );
plot( bearing2Axes, times, trueObservations(4,:) - sqrt(mleCov(4,4))', 'r-' );

plot( posErrAxes, times, mlePosErrs, 'r-' );

mleLikelihood = GaussianLogLikelihood(trueObsNoise, repmat(mleCov, [1, 1, T]), 'covariance' );
mleLikelihoodAvg = mean(mleLikelihood);
fprintf( 'MLE noise log likelihood: %f\n', mleLikelihoodAvg );

%% Run an EKF with no updates

odoEKF = ExtendedKalmanFilter( ekfTransFunc, ekfTransJac, ekfObsFunc, ekfObsJac, ekfObsDiff );
odoEKF.Initialize( trueStates(:,1), S0 );

odoStates = zeros(xDim,T);
odoStates(:,1) = odoEKF.x;
for t = 1:T
    odoEKF.Predict(trueControls(:,t), Q);
    odoStates(:,t) = odoEKF.x;
end
plot( trajectoryAxes, odoStates(1,:), odoStates(2,:), 'k-' );

%% Run an EKF with the true noise covariance

% tcovEKF = ExtendedKalmanFilter( ekfTransFunc, ekfTransJac, ekfObsFunc, ekfObsJac, ekfObsDiff );
% tcovEKF.Initialize( trueStates(:,1), S0 );
%
% tcovStates = zeros(xDim,T);
% tcovStates(:,1) = tcovEKF.x;
% for t = 1:T
%     tcovEKF.Update(noisedObservations(:,t), trueCov(:,:,t));
%     tcovStates(:,t) = tcovEKF.x;
%     tcovEKF.Predict(trueControls(:,t), Q);
% end
%
% tcovErrs = tcovStates - trueStates;
% tcovPosErrs = smooth( vnorm( tcovErrs(1:2,:) ), 20 );
%
% plot( trajectoryAxes, tcovStates(1,:), tcovStates(2,:), 'g-' );
%
% plot( rangeAxes, times, trueObservations(1,:) + sqrt(squeeze(trueCov(1,1,:)))', 'g-' );
% plot( rangeAxes, times, trueObservations(1,:) - sqrt(squeeze(trueCov(1,1,:)))', 'g-' );
% plot( bearingAxes, times, trueObservations(2,:) + sqrt(squeeze(trueCov(2,2,:)))', 'g-' );
% plot( bearingAxes, times, trueObservations(2,:) - sqrt(squeeze(trueCov(2,2,:)))', 'g-' );
%
% plot( range2Axes, times, trueObservations(3,:) + sqrt(squeeze(trueCov(3,3,:)))', 'g-' );
% plot( range2Axes, times, trueObservations(3,:) - sqrt(squeeze(trueCov(3,3,:)))', 'g-' );
% plot( bearing2Axes, times, trueObservations(4,:) + sqrt(squeeze(trueCov(4,4,:)))', 'g-' );
% plot( bearing2Axes, times, trueObservations(4,:) - sqrt(squeeze(trueCov(4,4,:)))', 'g-' );
%
% plot( posErrAxes, times, tcovPosErrs, 'g-' );

%% Fit positive basis covariance model
numBasis = 65;
pbParams.basis = zeros(zDim,zDim,numBasis);
pbParams.wPenalty = 1E-4;

for i = 2:2:numBasis
    A = mvnrnd( zeros(2,1), eye(2) )';
    pd = A*A';
    pd = pd/trace(pd); % Uniform trace-power
    pbParams.basis(1:2,1:2,i) = pd;
    pbParams.basis(3:4,3:4,i+1) = pd;
end

pbParams.basis(:,:,1) = 1E-6*eye(zDim);

pbParams.w = 0*ones(fDim, numBasis);
pbParams.modelMode = 'covariance';
pbParams.dataMode = 'innovation';

pbDamping = repmat( 1E-6*eye(zDim), [1,1,T] );
% pbParams = PositiveBasisAscent( trueObsNoise, trueFeatures, pbDamping, pbParams, ascentParams );
pbObjectiveFunction = @(p) PositiveBasisObjectiveFunction( p, pbParams, ...
    trueObsNoise, trueFeatures, pbDamping );

options = optimoptions( 'fminunc', 'algorithm', 'quasi-newton', 'GradObj','on', ...
    'diagnostics', 'off', 'derivativecheck', 'off', ...
    'plotfcns', {@optimplotx, @optimplotfval}, ...
    'tolfun', 0 );
pbParamVec = fminunc( pbObjectiveFunction, zeros(1,fDim*numBasis), options );
pbParams.w = reshape( pbParamVec, size(pbParams.w) );

pbCov = PositiveBasisRegression( trueFeatures, pbParams ) + pbDamping;

plot( rangeAxes, times, trueObservations(1,:) + sqrt(squeeze(pbCov(1,1,:)))', 'k-' );
plot( rangeAxes, times, trueObservations(1,:) - sqrt(squeeze(pbCov(1,1,:)))', 'k-' );
plot( bearingAxes, times, trueObservations(2,:) + sqrt(squeeze(pbCov(2,2,:)))', 'k-' );
plot( bearingAxes, times, trueObservations(2,:) - sqrt(squeeze(pbCov(2,2,:)))', 'k-' );

plot( range2Axes, times, trueObservations(3,:) + sqrt(squeeze(pbCov(3,3,:)))', 'k-' );
plot( range2Axes, times, trueObservations(3,:) - sqrt(squeeze(pbCov(3,3,:)))', 'k-' );
plot( bearing2Axes, times, trueObservations(4,:) + sqrt(squeeze(pbCov(4,4,:)))', 'k-' );
plot( bearing2Axes, times, trueObservations(4,:) - sqrt(squeeze(pbCov(4,4,:)))', 'k-' );

pbEKF = ExtendedKalmanFilter( ekfTransFunc, ekfTransJac, ekfObsFunc, ekfObsJac, ekfObsDiff );
pbEKF.Initialize( trueStates(:,1), S0 );

pbStates = zeros(xDim,T);
pbStates(:,1) = pbEKF.x;
for t = 1:T
    pbEKF.Update(noisedObservations(:,t), pbCov(:,:,t));
    pbStates(:,t) = pbEKF.x;
    pbEKF.Predict(trueControls(:,t), Q);
end
plot( trajectoryAxes, pbStates(1,:), pbStates(2,:), 'k-' );

pbErrs = pbStates - trueStates;
pbPosErrs = smooth( vnorm( pbErrs(1:2,:) ), 20 );
plot( posErrAxes, times, pbPosErrs, 'k-' );

pbLikelihood = GaussianLogLikelihood( trueObsNoise, pbCov, 'covariance' );
pbLikelihoodAvg = mean(pbLikelihood);
fprintf( 'PB data log likelihood with %d bases: %f\n', numBasis, pbLikelihoodAvg );

%% Fit error covariance model
% gliFit = GaussianLikelihoodAscent( trueObsNoise, trueFeatures, pbDamping, ...
%     initParams, ascentParams );
ricParams.modelMode = 'covariance';
ricParams.dataMode = 'innovation';
ricParams.w = zeros(fDim,wDim);
ricParams.v = zeros(fDim,vDim);
ricParams.vPenalty = 1E-3;
ricParams.wPenalty = 0;

ricObjectiveFunction = @(p) ModifiedCholeskyObjectiveFunction( p, ricParams, ...
    trueObsNoise, trueFeatures, pbDamping );

options = optimoptions( 'fminunc', 'algorithm', 'quasi-newton', 'GradObj','on', ...
    'diagnostics', 'off', 'derivativecheck', 'off', ...
    'plotfcns', {@optimplotx, @optimplotfval}, ...
    'tolfun', 0 );
ricInitVec = [ricParams.v(:); ricParams.w(:)];
ricParamVec = fminunc( ricObjectiveFunction, ricInitVec, options );

ricParams.v = reshape( ricParamVec(1:fDim*vDim), fDim, vDim );
ricParams.w = reshape( ricParamVec(fDim*vDim + (1:fDim*wDim)), fDim, wDim );

ricCov = ModifiedCholeskyRegression( trueFeatures, ricParams, true ) + pbDamping;

plot( rangeAxes, times, trueObservations(1,:) + sqrt(squeeze(ricCov(1,1,:)))', 'm-' );
plot( rangeAxes, times, trueObservations(1,:) - sqrt(squeeze(ricCov(1,1,:)))', 'm-' );
plot( bearingAxes, times, trueObservations(2,:) + sqrt(squeeze(ricCov(2,2,:)))', 'm-' );
plot( bearingAxes, times, trueObservations(2,:) - sqrt(squeeze(ricCov(2,2,:)))', 'm-' );

plot( range2Axes, times, trueObservations(3,:) + sqrt(squeeze(ricCov(3,3,:)))', 'm-' );
plot( range2Axes, times, trueObservations(3,:) - sqrt(squeeze(ricCov(3,3,:)))', 'm-' );
plot( bearing2Axes, times, trueObservations(4,:) + sqrt(squeeze(ricCov(4,4,:)))', 'm-' );
plot( bearing2Axes, times, trueObservations(4,:) - sqrt(squeeze(ricCov(4,4,:)))', 'm-' );

ricEKF = ExtendedKalmanFilter( ekfTransFunc, ekfTransJac, ekfObsFunc, ekfObsJac, ekfObsDiff );
ricEKF.Initialize( trueStates(:,1), S0 );

ricStates = zeros(xDim,T);
ricStates(:,1) = ricEKF.x;
for t = 1:T
    ricEKF.Update(noisedObservations(:,t), ricCov(:,:,t));
    ricStates(:,t) = ricEKF.x;
    ricEKF.Predict(trueControls(:,t), Q);
end
plot( trajectoryAxes, ricStates(1,:), ricStates(2,:), 'm-' );

ricErrs = ricStates - trueStates;
ricPosErrs = smooth( vnorm( ricErrs(1:2,:) ), 20 );
plot( posErrAxes, times, ricPosErrs, 'm-' );

ricLikelihood = GaussianLogLikelihood( trueObsNoise, ricCov, 'covariance' );
ricLikelihoodAvg = mean(ricLikelihood);
fprintf( 'Damped covariance data log likelihood: %f\n', ricLikelihoodAvg );

%% Run GLI online

% onlineParams = initParams;
% onlineAscentParams = ascentParams;
% onlineAscentParams.maximumIterations = 1;
% onlineAscentParams.showPlots = false;
% onlineAscentParams.maximumBacktracks = 0;
% % onlineAscentParams.stepSize = 1E-1;
%
% onlineLosses = zeros(1,T);
%
% ogliEKF = ExtendedKalmanFilter( ekfTransFunc, ekfTransJac, ekfObsFunc, ekfObsJac, ekfObsDiff );
% ogliEKF.Initialize( trueStates(:,1), S0 );
%
% ogliStates = zeros(xDim,T);
% ogliStates(:,1) = ogliEKF.x;
% ogliCovs = zeros(zDim,zDim,T);
%
% for t = 1:T
%     ogliCovs(:,:,t) = inv(ModifiedCholeskyRegression(trueFeatures(:,t), onlineParams, true));
%     if max( eig(ogliCovs(:,:,t) ) > 1E1 )
%        blah = true;
%     end
%
%     ogliEKF.Update( noisedObservations(:,t), ogliCovs(:,:,t) );
%     ogliStates(:,t) = ogliEKF.x;
%     ogliEKF.Predict( trueControls(:,t), Q );
%
%     onlineAscentParams.stepSize = 1E-1/sqrt(t);
%     onlineParams = GLILikelihoodGradientAscent( trueObsNoise(:,t), trueFeatures(:,t), ...
%         onlineParams, onlineAscentParams );
% end
%
% ogliErrs = ogliStates - trueStates;
% ogliErrs(3,:) = wrapToPi( ogliErrs(3,:) );
% ogliPosErrs = smooth( vnorm( ogliErrs(1:2,:) ), 20 );
% plot( posErrAxes, times, ogliPosErrs, 'm--' );
%
% plot( trajectoryAxes, ogliStates(1,:), ogliStates(2,:), 'm--' );
%
% plot( rangeAxes, times, trueObservations(1,:) + sqrt(squeeze(ogliCovs(1,1,:)))', 'm--' );
% plot( rangeAxes, times, trueObservations(1,:) - sqrt(squeeze(ogliCovs(1,1,:)))', 'm--' );
% plot( bearingAxes, times, trueObservations(2,:) + sqrt(squeeze(ogliCovs(2,2,:)))', 'm--' );
% plot( bearingAxes, times, trueObservations(2,:) - sqrt(squeeze(ogliCovs(2,2,:)))', 'm--' );
%
% plot( range2Axes, times, trueObservations(3,:) + sqrt(squeeze(ogliCovs(3,3,:)))', 'm--' );
% plot( range2Axes, times, trueObservations(3,:) - sqrt(squeeze(ogliCovs(3,3,:)))', 'm--' );
% plot( bearing2Axes, times, trueObservations(4,:) + sqrt(squeeze(ogliCovs(4,4,:)))', 'm--' );
% plot( bearing2Axes, times, trueObservations(4,:) - sqrt(squeeze(ogliCovs(4,4,:)))', 'm--' );
%
% %% Run MLE online
%
% omleBuff = [];
% omleStates = zeros(xDim,T);
%
% omleCovs = zeros(zDim,zDim,T);
%
% omleEKF = ExtendedKalmanFilter( ekfTransFunc, ekfTransJac, ekfObsFunc, ekfObsJac, ekfObsDiff );
% omleEKF.Initialize( trueStates(:,1), S0 );
%
% omleStates(:,1) = omleEKF.x;
%
% % Give a few time steps to fill the MLE buffer a bit
% burnIn = 10;
% for i = 1:burnIn
%     omleBuff = [omleBuff, trueObsNoise(:,i)];
%     omleStates(:,i) = omleEKF.x;
%     omleEKF.Predict( trueControls(:,i), Q );
% end
%
% for i = burnIn+1:T
%     omleCovs(:,:,i) = cov( omleBuff' );
%     omleBuff = [omleBuff, trueObsNoise(:,i)];
%     omleEKF.Update( noisedObservations(:,i), omleCovs(:,:,i) );
%     omleStates(:,i) = omleEKF.x;
%     omleEKF.Predict( trueControls(:,i), Q );
% end
%
% plot( trajectoryAxes, omleStates(1,:), omleStates(2,:), 'r--' );
%
% omleErrs = omleStates - trueStates;
% omleErrs(3,:) = wrapToPi( omleErrs(3,:) );
% omlePosErrs = smooth( vnorm( omleErrs(1:2,:) ), 20 );
% plot( posErrAxes, times, omlePosErrs, 'r--' );
%
% % Show on observation plots
% plot( rangeAxes, times, trueObservations(1,:)' + sqrt(squeeze(omleCovs(1,1,:))), 'r--' );
% plot( rangeAxes, times, trueObservations(1,:)' - sqrt(squeeze(omleCovs(1,1,:))), 'r--' );
% plot( bearingAxes, times, trueObservations(2,:)' + sqrt(squeeze(omleCovs(2,2,:))), 'r--' );
% plot( bearingAxes, times, trueObservations(2,:)' - sqrt(squeeze(omleCovs(2,2,:))), 'r--' );
%
% plot( range2Axes, times, trueObservations(3,:)' + sqrt(squeeze(omleCovs(3,3,:))), 'r--' );
% plot( range2Axes, times, trueObservations(3,:)' - sqrt(squeeze(omleCovs(3,3,:))), 'r--' );
% plot( bearing2Axes, times, trueObservations(4,:)' + sqrt(squeeze(omleCovs(4,4,:))), 'r--' );
% plot( bearing2Axes, times, trueObservations(4,:)' - sqrt(squeeze(omleCovs(4,4,:))), 'r--' );
%
% %% Run GLI online on innovation
%
% onlineInnovationParams = initParams;
% onlineAscentParams = ascentParams;
% onlineAscentParams.maximumIterations = 1;
% onlineAscentParams.showPlots = false;
% onlineAscentParams.maximumBacktracks = 0;
% % onlineAscentParams.stepSize = 1E-1;
%
% onlineLosses = zeros(1,T);
%
% oigliEKF = ExtendedKalmanFilter( ekfTransFunc, ekfTransJac, ekfObsFunc, ekfObsJac, ekfObsDiff );
% oigliEKF.Initialize( trueStates(:,1), S0 );
%
% oigliStates = zeros(xDim,T);
% oigliStates(:,1) = oigliEKF.x;
% oigliCovs = zeros(zDim,zDim,T);
%
% oigliInnovations = zeros(zDim,T);
%
% for t = 1:T
%     zEst = ekfObsFunc( oigliEKF.x );
%     f = obsFeatFunc( zEst );
%     oigliCovs(:,:,t) = inv(LinearCovarianceModel( f, onlineInnovationParams));
%     [~, innovation] = oigliEKF.Update( noisedObservations(:,t), oigliCovs(:,:,t) );
%     oigliInnovations(:,t) = innovation;
%     oigliStates(:,t) = oigliEKF.x;
%     oigliEKF.Predict( trueControls(:,t), Q );
%
%     onlineAscentParams.stepSize = 1E-1/sqrt(t);
%     onlineInnovationParams = GLILikelihoodGradientAscent( innovation, f, ...
%         onlineInnovationParams, onlineAscentParams );
% end
%
% oigliErrs = oigliStates - trueStates;
% oigliErrs(3,:) = wrapToPi( oigliErrs(3,:) );
% oigliPosErrs = smooth( vnorm( oigliErrs(1:2,:) ), 20 );
% plot( posErrAxes, times, oigliPosErrs, 'm:' );
%
% plot( trajectoryAxes, oigliStates(1,:), oigliStates(2,:), 'm:' );
%
% plot( rangeAxes, times, trueObservations(1,:) + sqrt(squeeze(oigliCovs(1,1,:)))', 'm:' );
% plot( rangeAxes, times, trueObservations(1,:) - sqrt(squeeze(oigliCovs(1,1,:)))', 'm:' );
% plot( bearingAxes, times, trueObservations(2,:) + sqrt(squeeze(oigliCovs(2,2,:)))', 'm:' );
% plot( bearingAxes, times, trueObservations(2,:) - sqrt(squeeze(oigliCovs(2,2,:)))', 'm:' );
%
% plot( range2Axes, times, trueObservations(3,:) + sqrt(squeeze(oigliCovs(3,3,:)))', 'm:' );
% plot( range2Axes, times, trueObservations(3,:) - sqrt(squeeze(oigliCovs(3,3,:)))', 'm:' );
% plot( bearing2Axes, times, trueObservations(4,:) + sqrt(squeeze(oigliCovs(4,4,:)))', 'm:' );
% plot( bearing2Axes, times, trueObservations(4,:) - sqrt(squeeze(oigliCovs(4,4,:)))', 'm:' );
%
% %% Run MLE online on innovation
%
% oimleBuff = [];
% oimleStates = zeros(xDim,T);
%
% oimleCovs = zeros(zDim,zDim,T);
%
% oimleEKF = ExtendedKalmanFilter( ekfTransFunc, ekfTransJac, ekfObsFunc, ekfObsJac, ekfObsDiff );
% oimleEKF.Initialize( trueStates(:,1), S0 );
%
% oimleStates(:,1) = oimleEKF.x;
%
% % Give a few time steps to fill the MLE buffer a bit
% burnIn = 10;
% R0 = eye(zDim);
% for i = 1:burnIn
%     oimleStates(:,i) = oimleEKF.x;
%     oimleEKF.Predict( trueControls(:,i), Q );
%     [~, innovation] = oimleEKF.Update( noisedObservations(:,i), R0 );
%     oimleBuff = [oimleBuff, innovation];
% end
%
% for i = burnIn+1:T
%     oimleCovs(:,:,i) = cov( oimleBuff' );
%     [~, innovation] = oimleEKF.Update( noisedObservations(:,i), oimleCovs(:,:,i) );
%     oimleBuff = [oimleBuff, innovation];
%     oimleStates(:,i) = oimleEKF.x;
%     oimleEKF.Predict( trueControls(:,i), Q );
% end
%
% plot( trajectoryAxes, oimleStates(1,:), oimleStates(2,:), 'r:' );
%
% oimleErrs = oimleStates - trueStates;
% oimleErrs(3,:) = wrapToPi( oimleErrs(3,:) );
% oimlePosErrs = smooth( vnorm( oimleErrs(1:2,:) ), 20 );
% plot( posErrAxes, times, oimlePosErrs, 'r:' );
%
% % Show on observation plots
% plot( rangeAxes, times, trueObservations(1,:)' + sqrt(squeeze(oimleCovs(1,1,:))), 'r:' );
% plot( rangeAxes, times, trueObservations(1,:)' - sqrt(squeeze(oimleCovs(1,1,:))), 'r:' );
% plot( bearingAxes, times, trueObservations(2,:)' + sqrt(squeeze(oimleCovs(2,2,:))), 'r:' );
% plot( bearingAxes, times, trueObservations(2,:)' - sqrt(squeeze(oimleCovs(2,2,:))), 'r:' );
%
% plot( range2Axes, times, trueObservations(3,:)' + sqrt(squeeze(oimleCovs(3,3,:))), 'r:' );
% plot( range2Axes, times, trueObservations(3,:)' - sqrt(squeeze(oimleCovs(3,3,:))), 'r:' );
% plot( bearing2Axes, times, trueObservations(4,:)' + sqrt(squeeze(oimleCovs(4,4,:))), 'r:' );
% plot( bearing2Axes, times, trueObservations(4,:)' - sqrt(squeeze(oimleCovs(4,4,:))), 'r:' );