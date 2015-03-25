%% Load trials and run their filters

% For each trial, we fit:
%   1. Modifed cholesky model to innovation sequence
%   2. Modifed cholesky model with EM
%   3. Positive combination model to innovation sequence [pending]
%   4. Positive combination model with EM [pending]
%   5. Constant matrix to innovation sequence
%   6. Constant matrix with EM

numFolds = 4;
%% Generate final results from all folds
for n = 1:numFolds

    resultsName = ['../data/truck/Results_Fold', ...
        num2str(n), '.mat'];

    fprintf( ['Parsing results from ', resultsName, '\n'] );
    trial = load( resultsName );
    data = load( trial.dataName );
    
    trainTrials = data.folds(trial.testParams.trainInds);
    testTrials = data.folds(trial.testParams.testInds);

    trainEvalFunc = @(regressor) RunTruckResults( trainTrials, regressor );
    testEvalFunc = @(regressor) RunTruckResults( testTrials, regressor );

    baselineTrainFunc = @(est) RunTruckBaselines( trainTrials, est );
    baselineTestFunc = @(est) RunTruckBaselines( testTrials, est );
    estimator = WindowedCovarianceEstimator( 2, trial.baselineResults.bestWindow );

    fprintf( 'Evaluating baseline...\n' );
    baselineTemp = trial.baselineResults;
    baselineTemp.train_eval = EvaluateBaselineResults( baselineTrainFunc( estimator ) );
    baselineTemp.test_eval = EvaluateBaselineResults( baselineTestFunc( estimator ) );
    baselineResults(n) = baselineTemp;

    % 1. Modified cholesky with innovation sequence
    fprintf( 'Evaluating MCIR...\n' ); 
    mcirTemp = trial.mcirResults;
    mcirTemp.test_eval = EvaluateResults( testEvalFunc(mcirTemp.regressor) );
    mcirTemp.train_eval = EvaluateResults( trainEvalFunc(mcirTemp.regressor) );
    mcirResults(n) = mcirTemp;

    % 3. Modified cholesky with true errors
    fprintf( 'Evaluating MCTER...\n' );
    mcterTemp = trial.mcterResults;
    mcterTemp.test_eval = EvaluateResults( testEvalFunc(mcterTemp.regressor) );
    mcterTemp.train_eval = EvaluateResults( trainEvalFunc(mcterTemp.regressor) );
    mcterResults(n) = mcterTemp;

    % 4. Diagonal varying matrix with innovation sequence
    fprintf( 'Evaluating DCIR...\n' );
    dcirTemp = trial.dcirResults;
    dcirTemp.test_eval = EvaluateResults( testEvalFunc( dcirTemp.regressor ) );
    dcirTemp.train_eval = EvaluateResults( trainEvalFunc( dcirTemp.regressor ) );
    dcirResults(n) = dcirTemp;

    % 6. Constant matrix with true errors
    fprintf( 'Evaluating DCTER...\n' );
    dcterTemp = trial.dcterResults;
    dcterTemp.test_eval = EvaluateResults( testEvalFunc( dcterTemp.regressor ) );
    dcterTemp.train_eval = EvaluateResults( trainEvalFunc( dcterTemp.regressor ) );
    dcterResults(n) = dcterTemp;

    % 5. Constant matrix with EM
    fprintf( 'Evaluating CEMR...\n' );
    cemrTemp = trial.cemrResults;
    cemrTemp.test_eval = EvaluateResults( testEvalFunc( cemrTemp.regressor ) );
    cemrTemp.train_eval = EvaluateResults( trainEvalFunc( cemrTemp.regressor ) );
    cemrResults(n) = cemrTemp;

end
    
resultsName = '../data/truck/Results_Folded.mat';
    
save( resultsName, 'baselineResults', 'mcirResults', 'mcterResults', 'dcirResults', 'dcterResults', ...
      'cemrResults' );