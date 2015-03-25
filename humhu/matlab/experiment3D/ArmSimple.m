%% Load and divide up the arm data
dPath = '../data/abbcam/final0.mat';
data = LoadArmData( dPath );

zDim = 6;
fDim = size( ArmFeatures( SE3(rand(6,1)) ), 1 );
wDim = zDim;
vDim = zDim*(zDim-1)/2;

%% Estimate the tag pose
tagPoses = [data.cameraPose]*[data.tagMeasurement];
tagPoseMean = SE3( mean( tagPoses.GetCoordinates(), 2 ) );
tag.mean = tagPoseMean;
tagPoseErrs = tagPoseMean\tagPoses;
tag.covariance = cov( tagPoseErrs.GetCoordinates()' );

%% Approximately ientify the feature scales

%% Fit the models
damping = 1E-6*eye(6);

numPoints = numel( data );
numFolds = 4;
foldSize = numPoints/numFolds;
shuffleInds = randperm( numPoints );
shuffledData = data( shuffleInds );
foldInds = repmat( 1:numFolds, foldSize, 1 );
foldInds = foldInds(:);

for n = 1:numFolds

    fprintf( 'Executing fold %d...\n', n );
    
    trainPoints = shuffledData( foldInds ~= n );
    testPoints = shuffledData( foldInds == n );
    numTrainPoints = numel( trainPoints );
    numTestPoints = numel( testPoints );
    
    trainZPred = [trainPoints.cameraPose]\tag.mean;
    trainZErr = [trainPoints.tagMeasurement]/trainZPred;
    trainZErrSamples = trainZErr.GetCoordinates();
    
    testZPred = [testPoints.cameraPose]\tag.mean;
    testZErr = [testPoints.tagMeasurement]/testZPred;
    testZErrSamples = testZErr.GetCoordinates();
    
    trainFeatures = ArmFeatures( trainZPred );
    testFeatures = ArmFeatures( testZPred );
    fScales = max( abs(trainFeatures), [], 2 );

%     % 1. Fit modified cholesky with errors
%     mcir.type = 'modified_cholesky';
%     mcir.damping = damping;
%     mcir.modelMode = 'covariance';
%     mcir.matrixStructure = 'dense';
%     mcir.v = zeros( fDim, vDim );
%     mcir.w = zeros( fDim, wDim );
%     mcir.vPenalty = 0;
%     mcir.wPenalty = 0;
%     mcir.featureScales = fScales;
%     trainingParams.type = 'error';
%     
%     options = optimoptions( 'fminunc', 'algorithm', 'quasi-newton', 'GradObj','on', 'tolfun', 0, 'Display', 'off' );
%     [mcirResults.regressor(n), mcirResults.train] = FitRegressor( trainZErrSamples, trainFeatures, [], mcir, options );
%     mcirResults.trainR{n} = Regress( mcirResults.regressor(n), trainFeatures );
%     mcirResults.testR{n} = Regress( mcirResults.regressor(n), testFeatures );
%     mcirResults.train_ll(n,:) = GaussianLogLikelihood( trainZErrSamples, mcirResults.trainR{n}, 'covariance' )';
%     mcirResults.test_ll(n,:) = GaussianLogLikelihood( testZErrSamples, mcirResults.testR{n}, 'covariance' )';
%     mcirResults.avg_train_ll(n) = mean( mcirResults.train_ll(n,:) );
%     mcirResults.min_train_ll(n) = min( mcirResults.train_ll(n,:) );
%     mcirResults.avg_test_ll(n) = mean( mcirResults.test_ll(n,:) );
%     mcirResults.min_test_ll(n) = min( mcirResults.test_ll(n,:) );
%     fprintf( 'Dense MC train LL: %f testLL: %f\n', mcirResults.avg_train_ll(n), mcirResults.avg_test_ll(n) );
    
    % 4. Fit diagonal modified Cholesky with errors
%     dcir.type = 'modified_cholesky';
%     dcir.damping = damping;
%     dcir.modelMode = 'covariance';
%     dcir.matrixStructure = 'diagonal';
%     dcir.v = zeros( fDim, vDim );
%     dcir.w = zeros( fDim, wDim );
%     dcir.vPenalty = 0;
%     dcir.wPenalty = 0;
%     dcir.featureScales = fScales;
%     trainingParams.type = 'error';
%     [dcirResults.regressor(n), dcirResults.train] = FitRegressor( trainZErrSamples, trainFeatures, [], dcir, options );
%     dcirResults.trainR{n} = Regress( dcirResults.regressor(n), trainFeatures );
%     dcirResults.testR{n} = Regress( dcirResults.regressor(n), testFeatures );
%     dcirResults.train_ll(n,:) = GaussianLogLikelihood( trainZErrSamples, dcirResults.trainR{n}, 'covariance' )';
%     dcirResults.test_ll(n,:) = GaussianLogLikelihood( testZErrSamples, dcirResults.testR{n}, 'covariance' )';
%     dcirResults.avg_train_ll(n) = mean( dcirResults.train_ll(n,:) );
%     dcirResults.min_train_ll(n) = min( dcirResults.train_ll(n,:) );
%     dcirResults.avg_test_ll(n) = mean( dcirResults.test_ll(n,:) );
%     dcirResults.min_test_ll(n) = min( dcirResults.test_ll(n,:) );
%     fprintf( 'Diagonal MC train LL: %f testLL: %f\n', dcirResults.avg_train_ll(n), dcirResults.avg_test_ll(n) );
    
    % 5. Compare baseline MLE
    mleR{n} = cov( trainZErrSamples' );
    mleResults.trainR{n} = repmat( mleR{n}, [1,1,numTrainPoints] );
    mleResults.testR{n} = repmat( mleR{n}, [1,1,numTestPoints] );
    mleResults.train_ll(n,:) = GaussianLogLikelihood( trainZErrSamples, mleResults.trainR{n}, 'covariance' )';
    mleResults.test_ll(n,:) = GaussianLogLikelihood( testZErrSamples, mleResults.testR{n}, 'covariance' )';
    mleResults.avg_train_ll(n) = mean( mleResults.train_ll(n,:) );
    mleResults.min_train_ll(n) = min( mleResults.train_ll(n,:) );
    mleResults.avg_test_ll(n) = mean( mleResults.test_ll(n,:) );
    mleResults.min_test_ll(n) = min( mleResults.test_ll(n,:) );
    fprintf( 'MLE constant train LL: %f testLL: %f\n', mleResults.avg_train_ll(n), mleResults.avg_test_ll(n) );
    
end