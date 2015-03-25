function RunTruckTrial( testParams, dataName, resultsName )
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

% Determine training/testing sets
trials = load( dataName );
trainTrials = trials.folds(testParams.trainInds);

fprintf( ['Results will be saved to ', resultsName, '\n'] );

zDim = 2;
fDim = size( TruckFeatures( rand(zDim,1) ), 1 );
wDim = zDim;
vDim = zDim*(zDim-1)/2;

% 0. Fit baseline
% baselineEvalFunc = @(est) RunTruckBaselines( trainTrials, est );
% baselineParams.windows = round( logspace( log10(8), log10(100), 6 ) );
% baselineParams.zDim = zDim;
% baselineParams.numEstimators = trials.numLandmarks;
% baselineParams.initCov = trials.noiseCov;
% [baselineResults.bestWindow, baselineResults.train] = TrainBaseline( baselineEvalFunc, baselineParams );

fScales = trials.featureScales;
trainEvalFunc = @(regressors) RunTruckResults( trainTrials, regressors );
trainingParams.maxIters = 50;
trainingParams.delTolerance = 1E-4;
trainingParams.maxEMAttempts = 5;
trainingParams.emAlpha = 0.5;
damping = 1E-6*eye(2);
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

dcirs = repmat( dcir, trials.numLandmarks, 1 );
tParams = repmat( trainingParams, trials.numLandmarks, 1 );
tic;
[dcirResults.regressor, dcirResults.train] = TrainRegressor( trainEvalFunc, dcirs, tParams);
dcirResults.train.time = toc;

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

save( resultsName, 'mcirResults', 'mcterResults', ...
    'dcirResults', 'dcterResults', ...
    'cemrResults', 'baselineResults', ...
    'dataName', 'testParams' );
