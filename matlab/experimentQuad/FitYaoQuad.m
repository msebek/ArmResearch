%% TODO Move into a setup script
bagRoot = '~/Software/CovarianceRegression/data/yao_quad/bagfiles/';
bagFiles = {    'delta_141216_B', ...
    'hotel_141014_E', ...
    'hotel_141014_H', ...
    'hotel_141030_B', ...
    'hotel_141103_G', ...
    'hotel_141116_A', ...
    'hotel_141201_A' };
numBags = numel(bagFiles);

quadNames = { 'delta', 'hotel', 'hotel', 'hotel', 'hotel', 'hotel', 'hotel' };

matRoot = '~/Software/CovarianceRegression/data/yao_quad/mat/';

%% Display data

t0 = icp(1).timestamp;
altTimes = [alt.timestamp] - t0;
imuTimes = [imu.timestamp] - t0;
cfTimes = [cf.timestamp] - t0;
scanTimes = [scans.timestamp] - t0;
icpTimes = [icp.timestamp] - t0;
slamTimes = [slam.timestamp] - t0;

figure;
axes;
hold on;
icpErr = [icp.estimated] - [icp.observed];
plot( icpTimes, icpErr(1,:), 'r-' );
plot( icpTimes, icpErr(2,:), 'g-' );
plot( icpTimes, icpErr(3,:), 'b-' );
legend('x', 'y', '\theta', 'location', 'best');
title( 'ICP innovation' );
xlabel('Time');
ylabel('Innovation');

figure;
axes;
hold on;
altErr = [alt.estimated] - [alt.observed];
plot( altTimes, altErr(1,:), 'r-' );
title( 'Altitude innovation' );
xlabel('Time');
ylabel('Innovation');

figure;
axes;
hold on;
slamErr = [slam.estimated] - [slam.observed];
plot( slamTimes, slamErr(1,:), 'r-' );
plot( slamTimes, slamErr(2,:), 'g-' );
plot( slamTimes, slamErr(3,:), 'b-' );
legend('x', 'y', '\theta', 'location', 'best');
title( 'Absolute localization innovation' );
xlabel('Time');
ylabel('Innovation');

%% Fit Altitude model
% Features:
% I suspect the primary sources of error here are the quad pitching about
% and flying over unexpected obstacles. 

indexPad = 20;

% Interpolate CF, IMU
altAngVel = interp1( imuTimes, [imu.angularVelocity]', altTimes(indexPad:end-indexPad) )'; 
altLinAcc = interp1( imuTimes, [imu.linearAcceleration]', altTimes(indexPad:end-indexPad) )';
altMagnet = interp1( imuTimes, [imu.magnetometer]', altTimes(indexPad:end-indexPad) )';
altTemp = interp1( imuTimes, [imu.temperature]', altTimes(indexPad:end-indexPad) )';
altPressure = interp1( imuTimes, [imu.pressure]', altTimes(indexPad:end-indexPad) )';

cfEuler = interp1( cfTimes, [cf.euler]', altTimes(indexPad:end-indexPad) )';

% Altitude features are roll, pitch, roll*pitch
altFDim = 6 + 1;

altParams.modelMode = 'covariance';
altParams.dataMode = 'error';
altParams.w = zeros( altFdim, 1 );

%% Fit ICP model
indexPad = 20;
icpValid = icp(indexPad:end-indexPad);
icpValidTimes = [icpValid.timestamp] - t0;

icpN = numel( icpValidTimes );
icpDim = 3;
icpVDim = (icpDim-1)*icpDim/2;
icpWDim = icpDim;

icpAngVel = interp1( imuTimes, [imu.angularVelocity]', icpValidTimes )'; 
icpLinAcc = interp1( imuTimes, [imu.linearAcceleration]', icpValidTimes )';
icpMagnet = interp1( imuTimes, [imu.magnetometer]', icpValidTimes )';
icpTemp = interp1( imuTimes, [imu.temperature]', icpValidTimes );
icpPressure = interp1( imuTimes, [imu.pressure]', icpValidTimes )';

icpEuler = interp1( cfTimes, [cf.euler]', icpValidTimes )';

icpFeatures = [ log(abs(icpAngVel));
                log(abs(icpLinAcc)/10);
                %log(abs(icpMagnet));
                %log(icpTemp/30);
                %log(icpPressure/1000);
                log(abs(icpEuler));
                ones(1,icpN) ];

icpErrs = [icpValid.estimated] - [icpValid.observed];
icpErrs(3,:) = wrapToPi( icpErrs(3,:) );
            
% Altitude features are roll, pitch, roll*pitch
icpFDim = 9 + 1;

icpPrioris = reshape( [icpValid.mappedPoseCovariance], 3, 3, icpN );

icpParams.modelMode = 'covariance';
icpParams.dataMode = 'innovation';
icpParams.w = zeros( icpFDim, icpWDim );
icpParams.v = zeros( icpFDim, icpVDim );
icpParams.vPenalty = 1E-4;
icpParams.wPenalty = 0;

icpDamping = repmat( 1E-6*eye(3,3), [1,1,icpN] );
icpObjectiveFunction = @(p) ModifiedCholeskyObjectiveFunction( p, icpParams, ...
    icpErrs, icpFeatures, icpDamping );

icpOptions = optimoptions( 'fminunc', 'algorithm', 'quasi-newton', 'GradObj','on', ...
        'diagnostics', 'off', 'derivativecheck', 'off', ...
        'plotfcns', {@optimplotx, @optimplotfval}, ...
        'tolfun', 0 );
icpInitVec = [icpParams.v(:); icpParams.w(:)];
icpParamVec = fminunc( icpObjectiveFunction, icpInitVec, icpOptions );

icpParams.v = reshape( icpParamVec(1:icpFDim*icpVDim), icpFDim, icpVDim );
icpParams.w = reshape( icpParamVec(icpFDim*icpVDim + (1:icpFDim*icpWDim)), icpFDim, icpWDim );

icpMLER = cov( icpErrs' );
icpR = ModifiedCholeskyRegression( icpFeatures, icpParams, true );
icpOrigR = reshape( [icpValid.measurementCovariance], 3, 3, icpN );

icpMLELL = GaussianLogLikelihood( icpErrs, repmat(icpMLER, [1,1,icpN]), 'covariance' );
icpOrigLL = GaussianLogLikelihood( icpErrs, icpOrigR + icpPrioris, 'covariance' );
icpLL = GaussianLogLikelihood( icpErrs, icpR + icpDamping, 'covariance' );

fprintf( 'ICP MLE LL: %f\n', mean(icpMLELL) );
fprintf( 'ICP Censi LL: %f\n', mean(icpOrigLL) );
fprintf( 'ICP Fit LL: %f\n', mean(icpLL) );

%% Plot results
figure;
axes;
hold on;

plot( icpValidTimes, abs(icpErrs(1,:)), '.' );
plot( icpValidTimes, squeeze(sqrt(icpR(1,1,:))), 'm-' );
plot( icpValidTimes, squeeze(sqrt(icpR(1,1,:))), 'm-' );

figure;
axes;
hold on;

plot( icpValidTimes, abs(icpErrs(2,:)), '.' );
plot( icpValidTimes, squeeze(sqrt(icpR(2,2,:))), 'm-' );
plot( icpValidTimes, squeeze(sqrt(icpR(2,2,:))), 'm-' );

figure;
axes;
hold on;

plot( icpValidTimes, abs(icpErrs(3,:)), '.' );
plot( icpValidTimes, squeeze(sqrt(icpR(3,3,:))), 'm-' );
plot( icpValidTimes, squeeze(sqrt(icpR(3,3,:))), 'm-' );