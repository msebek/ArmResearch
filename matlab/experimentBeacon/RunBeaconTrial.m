function RunBeaconTrial( testParams, trialRoot, postfix )
% Runs a fold on the specified params
% testParams has fields:
%   nBeacons - Number of beacons
%   nIters - Number of trials run
%   testInds - Indices to test on
%   trainInds - Indices to train on

% For each trial, we fit:
%   1. Modifed cholesky model to innovation sequence
%   2. Modifed cholesky model with EM
%   2.5 Modified cholesky model with true errors
%   5. Constant matrix to innovation sequence
%   6. Constant matrix with EM
%   6.5 Constant matrix with true errors
%   7. Diagonal constant matrix with innovation sequence
%   8. Diagonal constant matrix with EM
%   8.5. Diagonal matrix with true errors

nBeacons = testParams.nBeacons;

% Determine training/testing sets
trainIters = testParams.trainInds;
testIters = testParams.testInds;
numTrainIters = numel(trainIters);
numTestIters = numel(testIters);

fprintf( 'Beginning trials for %d beacons...\n', nBeacons );

resultsName = [trialRoot, 'BeaconCartSim_', num2str(nBeacons), 'Beacons_Results', ...
    postfix, '.mat'];
fprintf( ['Results will be saved to ', resultsName, '\n'] );

trainNames = cell(1,numTrainIters);
testNames = cell(1,numTestIters);

fprintf( 'Training on datasets:\n' );
for iter = 1:numTrainIters
    trialName = [trialRoot, 'BeaconCartSim_', num2str(nBeacons), ...
        'Beacons_Trial', num2str(trainIters(iter)), '.mat'];
    trainNames{iter} = trialName;
    fprintf( ['\t', trialName, '\n'] );
    
    if iter == 1
        trainTrials = load( trialName );
    else
        trainTrials(iter) = load( trialName );
    end
end
fprintf( 'Testing on datasets:\n' );
for iter = 1:numTestIters
    trialName = [trialRoot, 'BeaconCartSim_', num2str(nBeacons), ...
        'Beacons_Trial', num2str(testIters(iter)), '.mat'];
    testNames{iter} = trialName;
    fprintf( ['\t', trialName, '\n'] );
    
    if iter == 1
        testTrials = load( trialName );
    else
        testTrials(iter) = load( trialName );
    end
end


zDim = 2*nBeacons;
fDim = size( BeaconCartFeatures( rand(zDim,1) ), 1 );
wDim = zDim;
vDim = zDim*(zDim-1)/2;

% 0. Fit baseline
baselineEvalFunc = @(est) RunBeaconBaselines( trainTrials, est );
baselineParams.windows = round( logspace( log10(2*nBeacons), 1.5, 6 ) );
baselineParams.zDim = zDim;
[baselineResults.bestWindow, baselineResults.train] = TrainBaseline( baselineEvalFunc, baselineParams );

fScales = testParams.featureScales;
trainEvalFunc = @(regressor) RunBeaconResults( trainTrials, regressor );
trainingParams.maxIters = 50;
trainingParams.delTolerance = 1E-4;
trainingParams.maxEMAttempts = 5;
trainingParams.emAlpha = 0.5;
damping = 1E-6*eye(2*nBeacons);
vPenalty = 1E-3;
wPenalty = 1E-3;

% 4. Fit diagonal modified Cholesky with innovation
dcir.type = 'modified_cholesky';
dcir.damping = damping;
dcir.modelMode = 'covariance';
dcir.matrixStructure = 'diagonal';
dcir.v = zeros( fDim, vDim );
dcir.w = zeros( fDim, wDim );
dcir.vPenalty = vPenalty;
dcir.wPenalty = wPenalty;
dcir.featureScales = fScales;
trainingParams.type = 'innovation';
tic;
[dcirResults.regressor, dcirResults.train] = TrainRegressor( trainEvalFunc, dcir, trainingParams );
dcirResults.train.time = toc;

% 4.25 Fit diagonal modified Cholesky with EM
% dcemr.type = 'modified_cholesky';
% dcemr.damping = damping;
% dcemr.modelMode = 'covariance';
% dcemr.matrixStructure = 'diagonal';
% dcemr.v = zeros( fDim, vDim );
% dcemr.w = zeros( fDim, wDim );
% dcemr.vPenalty = 0;
% dcemr.wPenalty = 0;
% dcemr.featureScales = fScales;
% trainingParams.type = 'EM';
% [dcemrResults.regressor, dcemrResults.train] = TrainRegressor( trainEvalFunc, dcemr, trainingParams );

% 4.5 Fit diagonal modified Cholesky with true error
dcter.type = 'modified_cholesky';
dcter.damping = damping;
dcter.modelMode = 'covariance';
dcter.matrixStructure = 'diagonal';
dcter.v = dcir.v;
dcter.w = dcir.w;
% dcter.v = zeros( fDim, vDim );
% dcter.w = zeros( fDim, wDim );
dcter.vPenalty = vPenalty;
dcter.wPenalty = wPenalty;
dcter.featureScales = fScales;
trainingParams.type = 'error';
tic;
[dcterResults.regressor, dcterResults.train] = TrainRegressor( trainEvalFunc, dcter, trainingParams );
dcterResults.train.time = toc;

%1. Fit modified cholesky with innovation sequence
mcir.type = 'modified_cholesky';
mcir.damping = damping;
mcir.modelMode = 'covariance';
mcir.matrixStructure = 'dense';
% mcir.v = zeros( fDim, vDim );
% mcir.w = zeros( fDim, wDim );
mcir.v = dcirResults.regressor.v;
mcir.w = dcirResults.regressor.w;
mcir.vPenalty = vPenalty;
mcir.wPenalty = wPenalty;
mcir.featureScales = fScales;
trainingParams.type = 'innovation';
tic;
[mcirResults.regressor, mcirResults.train] = TrainRegressor( trainEvalFunc, mcir, trainingParams );
mcirResults.train.time = toc;

% 2. Fit modified cholesky with EM
% mcemr.type = 'modified_cholesky';
% mcemr.damping = damping;
% mcemr.modelMode = 'covariance';
% mcemr.matrixStructure = 'dense';
% % mcemr.v = zeros( fDim, vDim );
% % mcemr.w = zeros( fDim, wDim );
% mcemr.v = dcemrResults.regressor.v;
% mcemr.w = dcemrResults.regressor.w;
% mcemr.vPenalty = 0;
% mcemr.wPenalty = 0;
% mcemr.featureScales = fScales;
% trainingParams.type = 'EM';
% tic;
% [mcemrResults.regressor, mcemrResults.train] = TrainRegressor( trainEvalFunc, mcemr, trainingParams );
% mcemrResults.train.time = toc;

% 3. Fit modified Cholesky with true errors
mcter.type = 'modified_cholesky';
mcter.damping = damping;
mcter.modelMode = 'covariance';
mcter.matrixStructure = 'dense';
mcter.v = mcirResults.regressor.v;
mcter.w = mcirResults.regressor.w;
% mcter.v = zeros( fDim, vDim );
% mcter.w = zeros( fDim, wDim );
mcter.vPenalty = vPenalty;
mcter.wPenalty = wPenalty;
mcter.featureScales = fScales;
trainingParams.type = 'error';
tic;
[mcterResults.regressor, mcterResults.train] = TrainRegressor( trainEvalFunc, mcter, trainingParams );
mcterResults.train.time = toc;


% % 5. Fit constant matrix with innovation sequence
% cir.type = 'constant';
% cir.damping = damping;
% cir.modelMode = 'covariance';
% cir.matrixStructure = 'dense';
% cir.v = zeros( 1, vDim );
% cir.w = zeros( 1, wDim );
% cir.vPenalty = 0;
% cir.wPenalty = 0;
% cir.featureScales = 1;
% cir.R = ModifiedCholeskyRegression( 1, cir ) + cir.damping;
% trainingParams.type = 'innovation';
% tic;
% [cirResults.regressor, cirResults.train] = TrainRegressor( trainEvalFunc, cir, trainingParams );
% cirResults.train.time = toc;

% 6. Fit constant matrix with EM
cemr.type = 'constant';
cemr.damping = damping;
cemr.modelMode = 'covariance';
cemr.matrixStructure = 'dense';
cemr.v = zeros( 1, vDim );
cemr.w = zeros( 1, wDim );
cemr.vPenalty = 0;
cemr.wPenalty = 0;
cemr.featureScales = 1;
cemr.R = ModifiedCholeskyRegression( 1, cemr ) + cemr.damping;
trainingParams.type = 'mle';
tic;
[cemrResults.regressor, cemrResults.train] = TrainRegressor( trainEvalFunc, cemr, trainingParams );
cemrResults.train.time = toc;

% 6.5. Fit constant matrix with true errors
% cter.type = 'constant';
% cter.damping = damping;
% cter.modelMode = 'covariance';
% cter.matrixStructure = 'dense';
% cter.v = cirResults.regressor.v;
% cter.w = cirResults.regressor.w;
% cter.vPenalty = 0;
% cter.wPenalty = 0;
% cter.featureScales = 1;
% cter.R = ModifiedCholeskyRegression( 1, cter ) + cter.damping;
% trainingParams.type = 'error';
% tic;
% [cterResults.regressor, cterResults.train] = TrainRegressor( trainEvalFunc, cter, trainingParams );
% cterResults.train.time = toc;

% 7. Fit constant diagonal matrix with innovation sequence
% dir.type = 'constant';
% dir.damping = damping;
% dir.modelMode = 'covariance';
% dir.matrixStructure = 'diagonal';
% dir.v = zeros( 1, vDim );
% dir.w = zeros( 1, wDim );
% dir.vPenalty = 0;
% dir.wPenalty = 0;
% dir.featureScales = 1;
% dir.R = ModifiedCholeskyRegression( 1, dir ) + dir.damping;
% trainingParams.type = 'innovation';
% tic;
% [dirResults.regressor, dirResults.train] = TrainRegressor( trainEvalFunc, dir, trainingParams );
% dirResults.train.time = toc;

% 8. Fit constant diagonal matrix with EM
% demr.type = 'constant';
% demr.damping = damping;
% demr.modelMode = 'covariance';
% demr.matrixStructure = 'diagonal';
% demr.v = zeros( 1, vDim );
% demr.w = zeros( 1, wDim );
% demr.vPenalty = 0;
% demr.wPenalty = 0;
% demr.featureScales = 1;
% demr.R = ModifiedCholeskyRegression( 1, demr ) + demr.damping;
% trainingParams.type = 'EM';
% [demrResults.regressor, demrResults.train] = TrainRegressor( trainEvalFunc, demr, trainingParams );

% 8.5. Fit constant diagonal matrix with true errors
% dter.type = 'constant';
% dter.damping = damping;
% dter.modelMode = 'covariance';
% dter.matrixStructure = 'diagonal';
% % dter.v = dirResults.regressor.v;
% % dter.w = dirResults.regressor.w;
% dter.v = zeros( 1, vDim );
% dter.w = zeros( 1, wDim );
% dter.vPenalty = 0;
% dter.wPenalty = 0;
% dter.featureScales = 1;
% dter.R = ModifiedCholeskyRegression( 1, dter ) + dter.damping;
% trainingParams.type = 'error';
% tic;
% [dterResults.regressor, dterResults.train] = TrainRegressor( trainEvalFunc, dter, trainingParams );
% dterResults.train.time = toc;

% save( resultsName, 'mcirResults', 'mcemrResults', 'mcterResults', ...
%     'dcirResults', 'dcemrResults', 'dcterResults', ...
%     'cirResults', 'cemrResults', 'cterResults', ...
%     'dirResults', 'demrResults', 'dterResults', ...
%     'baselineResults', ...
%     'trainNames', 'testNames' );
save( resultsName, 'mcirResults', 'mcterResults', ...
    'dcirResults', 'dcterResults', ...
    'cemrResults', 'baselineResults', ...
    'trainNames', 'testNames' );
