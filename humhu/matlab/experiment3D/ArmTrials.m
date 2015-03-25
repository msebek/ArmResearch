%% Load and divide up the arm data
dPath = '../data/abbcam/final0.mat';
data = LoadArmData( dPath );

numPoints = numel( data );

trainRatio = 0.6; % 60% train
numTrainPoints = round( trainRatio*numPoints );
numTestPoints = numPoints - numTrainPoints;
shuffledInds = randperm( numPoints );
trainInds = shuffledInds( 1:numTrainPoints );
testInds = shuffledInds( numTrainPoints+1:end );

trainPoints = data( trainInds );
testPoints = data( testInds );
transitionNoise = 1E-3*eye(6);
trainTrajectory = GenerateArmTrajectories( trainPoints, transitionNoise );
testTrajectory = GenerateArmTrajectories( testPoints, transitionNoise );

zDim = 6;
fDim = size( ArmFeatures( SE3(rand(6,1)) ), 1 );
wDim = zDim;
vDim = zDim*(zDim-1)/2;

%% Approximately ientify the feature scales in the training trajectory
fScales = zeros( fDim, 1 );
for i = 1:numTrainPoints
    f = ArmFeatures( trainTrajectory(i).tagMeasurement );
    fScales( f > fScales ) = f( f > fScales );
end

trainingParams.maxIters = 40;
trainingParams.delTolerance = 1E-4;

emParams.maxIters = 5;
emParams.tagDel = 1E-3;
damping = 1E-6*eye(6);

% 1. Fit modified cholesky with innovation sequence
mcir.type = 'modified_cholesky';
mcir.damping = damping;
mcir.modelMode = 'covariance';
mcir.matrixStructure = 'dense';
mcir.v = zeros( fDim, vDim );
mcir.w = zeros( fDim, wDim );
mcir.vPenalty = 0;
mcir.wPenalty = 0;
mcir.featureScales = fScales;
trainingParams.type = 'innovation';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

% 3. Fit modified Cholesky with true errors
mcter.type = 'modified_cholesky';
mcter.damping = damping;
mcter.modelMode = 'covariance';
mcter.matrixStructure = 'dense';
mcter.v = mcirResults.regressor.v;
mcter.w = mcirResults.regressor.w;
mcter.vPenalty = 0;
mcter.wPenalty = 0;
mcter.featureScales = fScales;
trainingParams.type = 'error';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

% 4. Fit diagonal modified Cholesky with innovation
dcir.type = 'modified_cholesky';
dcir.damping = damping;
dcir.modelMode = 'covariance';
dcir.matrixStructure = 'diagonal';
dcir.v = zeros( fDim, vDim );
dcir.w = zeros( fDim, wDim );
dcir.vPenalty = 0;
dcir.wPenalty = 0;
dcir.featureScales = fScales;
trainingParams.type = 'innovation';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

% 4.25 Fit diagonal modified Cholesky with EM
dcemr.type = 'modified_cholesky';
dcemr.damping = damping;
dcemr.modelMode = 'covariance';
dcemr.matrixStructure = 'diagonal';
dcemr.v = zeros( fDim, vDim );
dcemr.w = zeros( fDim, wDim );
dcemr.vPenalty = 0;
dcemr.wPenalty = 0;
dcemr.featureScales = fScales;
trainingParams.type = 'EM';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

% 4.5 Fit diagonal modified Cholesky with true errpr
dcter.type = 'modified_cholesky';
dcter.damping = damping;
dcter.modelMode = 'covariance';
dcter.matrixStructure = 'diagonal';
dcter.v = dcir.v;
dcter.w = dcir.w;
dcter.vPenalty = 0;
dcter.wPenalty = 0;
dcter.featureScales = fScales;
trainingParams.type = 'error';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

% 5. Fit constant matrix with innovation sequence
cir.type = 'constant';
cir.damping = damping;
cir.modelMode = 'covariance';
cir.matrixStructure = 'dense';
cir.v = zeros( 1, vDim );
cir.w = zeros( 1, wDim );
cir.vPenalty = 0;
cir.wPenalty = 0;
cir.featureScales = 1;
cir.R = ModifiedCholeskyRegression( 1, cir ) + cir.damping;
trainingParams.type = 'innovation';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

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
trainingParams.type = 'EM';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

% 6.5. Fit constant matrix with true errors
cter.type = 'constant';
cter.damping = damping;
cter.modelMode = 'covariance';
cter.matrixStructure = 'dense';
cter.v = cirResults.regressor.v;
cter.w = cirResults.regressor.w;
cter.vPenalty = 0;
cter.wPenalty = 0;
cter.featureScales = 1;
cter.R = ModifiedCholeskyRegression( 1, cter ) + cter.damping;
trainingParams.type = 'error';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

% 7. Fit constant diagonal matrix with innovation sequence
dir.type = 'constant';
dir.damping = damping;
dir.modelMode = 'covariance';
dir.matrixStructure = 'diagonal';
dir.v = zeros( 1, vDim );
dir.w = zeros( 1, wDim );
dir.vPenalty = 0;
dir.wPenalty = 0;
dir.featureScales = 1;
dir.R = ModifiedCholeskyRegression( 1, dir ) + dir.damping;
trainingParams.type = 'innovation';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

% 8. Fit constant diagonal matrix with EM
demr.type = 'constant';
demr.damping = damping;
demr.modelMode = 'covariance';
demr.matrixStructure = 'diagonal';
demr.v = zeros( 1, vDim );
demr.w = zeros( 1, wDim );
demr.vPenalty = 0;
demr.wPenalty = 0;
demr.featureScales = 1;
demr.R = ModifiedCholeskyRegression( 1, demr ) + demr.damping;
trainingParams.type = 'EM';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

% 8.5. Fit constant diagonal matrix with true errors
dter.type = 'constant';
dter.damping = damping;
dter.modelMode = 'covariance';
dter.matrixStructure = 'diagonal';
dter.v = dirResults.regressor.v;
dter.w = dirResults.regressor.w;
dter.vPenalty = 0;
dter.wPenalty = 0;
dter.featureScales = 1;
dter.R = ModifiedCholeskyRegression( 1, dter ) + dter.damping;
trainingParams.type = 'error';
[mcirTag, mcirRegRes] = RunArmTrial( trainTrajectory, mcir, trainingParams, emParams );

save( resultsName, 'mcirResults', 'mcemrResults', 'mcterResults', ...
    'dcirResults', 'dcemrResults', 'dcterResults', ...
    'cirResults', 'cemrResults', 'cterResults', ...
    'dirResults', 'demrResults', 'dterResults', ...
    'baselineResults', ...
    'trainNames', 'testNames' );
