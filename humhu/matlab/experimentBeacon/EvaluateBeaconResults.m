%% Load trials and run their filters

% trialRoot = 'C:/Users/Humhu/Dropbox/Research/Papers/rss2015/data/beaconTrials/';
trialRoot = '../data/beaconTrials/';
load( [trialRoot, 'BeaconDatasetParams.mat' ] );

% For each trial, we fit:
%   1. Modifed cholesky model to innovation sequence
%   2. Modifed cholesky model with EM
%   3. Positive combination model to innovation sequence [pending]
%   4. Positive combination model with EM [pending]
%   5. Constant matrix to innovation sequence
%   6. Constant matrix with EM

numParamSets = numel( testParams );
numFolds = 4;
%%
for i = 1:numParamSets
    
    clear baselineResults mcirResults mcterResults dcirResults dcterResults cemrResults
    nBeacons = testParams(i).nBeacons;
    
    parfor n = 1:numFolds
        
        resultsName = [trialRoot, 'BeaconCartSim_', num2str(nBeacons), 'Beacons_Results_Fold', ...
            num2str(n), '.mat'];
        
        fprintf( ['Parsing results from ', resultsName, '\n'] );
        trial = load( resultsName );
        
        trainNames = trial.trainNames;
        testNames = trial.testNames;
        
        fprintf( 'Training on datasets:\n' );
        for j = 1:numel(trainNames)
            trialName = trainNames{j};
            fprintf( ['\t', trialName, '\n'] );
            
            if j == 1
                trainTrials = load( trialName );
            else
                trainTrials(i) = load( trialName );
            end
        end
        fprintf( 'Testing on datasets:\n' );
        for j = 1:numel(testNames)
            trialName = testNames{j};
            fprintf( ['\t', trialName, '\n'] );
            
            if iter == 1
                testTrials = load( trialName );
            else
                testTrials(j) = load( trialName );
            end
        end
        trainEvalFunc = @(regressor) RunBeaconResults( trainTrials, regressor );
        testEvalFunc = @(regressor) RunBeaconResults( testTrials, regressor );
        
        baselineTrainFunc = @(est) RunBeaconBaselines( trainTrials, est );
        baselineTestFunc = @(est) RunBeaconBaselines( testTrials, est );
        estimator = WindowedCovarianceEstimator( 2*nBeacons, trial.baselineResults.bestWindow );
        
        baselineTemp = trial.baselineResults;
        baselineTemp.train_eval = EvaluateBaselineResults( baselineTrainFunc( estimator ) );
        baselineTemp.test_eval = EvaluateBaselineResults( baselineTestFunc( estimator ) );
        baselineResults(n) = baselineTemp;
        
        % 1. Modified cholesky with innovation sequence
        mcirTemp = trial.mcirResults;
        mcirTemp.test_eval = EvaluateResults( testEvalFunc(mcirTemp.regressor) );
        mcirTemp.train_eval = EvaluateResults( trainEvalFunc(mcirTemp.regressor) );
        mcirResults(n) = mcirTemp;
        
        % 3. Modified cholesky with true errors
        mcterTemp = trial.mcterResults;
        mcterTemp.test_eval = EvaluateResults( testEvalFunc(mcterTemp.regressor) );
        mcterTemp.train_eval = EvaluateResults( trainEvalFunc(mcterTemp.regressor) );
        mcterResults(n) = mcterTemp;
        
        % 4. Diagonal varying matrix with innovation sequence
        dcirTemp = trial.dcirResults;
        dcirTemp.test_eval = EvaluateResults( testEvalFunc( dcirTemp.regressor ) );
        dcirTemp.train_eval = EvaluateResults( trainEvalFunc( dcirTemp.regressor ) );
        dcirResults(n) = dcirTemp;
        
        % 6. Constant matrix with true errors
        dcterTemp = trial.dcterResults;
        dcterTemp.test_eval = EvaluateResults( testEvalFunc( dcterTemp.regressor ) );
        dcterTemp.train_eval = EvaluateResults( trainEvalFunc( dcterTemp.regressor ) );
        dcterResults(n) = dcterTemp;
        
        % 5. Constant matrix with EM
        cemrTemp = trial.cemrResults;
        cemrTemp.test_eval = EvaluateResults( testEvalFunc( cemrTemp.regressor ) );
        cemrTemp.train_eval = EvaluateResults( trainEvalFunc( cemrTemp.regressor ) );
        cemrResults(n) = cemrTemp;
        
    end
    
    resultsName = [trialRoot, 'BeaconCartSim_', num2str(nBeacons), 'Beacons_Results_Folded.mat'];
    
    save( resultsName, 'baselineResults', 'mcirResults', 'mcterResults', 'dcirResults', 'dcterResults', ...
        'cemrResults' );
end

%% Calculate split results

i = 2;
foldSize = testParams(i).nIters/numFolds;
numSplits = testParams(i).nIters - foldSize;
clear baselineResults mcirResults mcterResults dcirResults dcterResults cemrResults
for k = 1:3:numSplits
    nBeacons = testParams(i).nBeacons;
    
    parfor n = 1:numFolds
        
        resultsName = [trialRoot, 'BeaconCartSim_', num2str(nBeacons), 'Beacons_Results_Split', ...
            num2str(k), '_Fold', num2str(n), '.mat'];
        
        fprintf( ['Parsing results from ', resultsName, '\n'] );
        trial = load( resultsName );
        
        trainNames = trial.trainNames;
        testNames = trial.testNames;
        trainEvalFunc = @(regressor) RunBeaconResults( trainNames, regressor );
        testEvalFunc = @(regressor) RunBeaconResults( testNames, regressor );
        
        baselineTrainFunc = @(est) RunBeaconBaselines( trainNames, est );
        baselineTestFunc = @(est) RunBeaconBaselines( testNames, est );
        estimator = WindowedCovarianceEstimator( 2*nBeacons, trial.baselineResults.bestWindow );
        
        baselineTemp = trial.baselineResults;
        baselineTemp.train_eval = EvaluateBaselineResults( baselineTrainFunc( estimator ) );
        baselineTemp.test_eval = EvaluateBaselineResults( baselineTestFunc( estimator ) );
        baselineResults(k,n) = baselineTemp;
        
        % 1. Modified cholesky with innovation sequence
        mcirTemp = trial.mcirResults;
        mcirTemp.test_eval = EvaluateResults( testEvalFunc(mcirTemp.regressor) );
        mcirTemp.train_eval = EvaluateResults( trainEvalFunc(mcirTemp.regressor) );
        mcirResults(k,n) = mcirTemp;
        
        % 3. Modified cholesky with true errors
        mcterTemp = trial.mcterResults;
        mcterTemp.test_eval = EvaluateResults( testEvalFunc(mcterTemp.regressor) );
        mcterTemp.train_eval = EvaluateResults( trainEvalFunc(mcterTemp.regressor) );
        mcterResults(k,n) = mcterTemp;
        
        % 4. Diagonal varying matrix with innovation sequence
        dcirTemp = trial.dcirResults;
        dcirTemp.test_eval = EvaluateResults( testEvalFunc( dcirTemp.regressor ) );
        dcirTemp.train_eval = EvaluateResults( trainEvalFunc( dcirTemp.regressor ) );
        dcirResults(k,n) = dcirTemp;
        
        % 6. Constant matrix with true errors
        dcterTemp = trial.dcterResults;
        dcterTemp.test_eval = EvaluateResults( testEvalFunc( dcterTemp.regressor ) );
        dcterTemp.train_eval = EvaluateResults( trainEvalFunc( dcterTemp.regressor ) );
        dcterResults(k,n) = dcterTemp;
        
        % 5. Constant matrix with EM
        cemrTemp = trial.cemrResults;
        cemrTemp.test_eval = EvaluateResults( testEvalFunc( cemrTemp.regressor ) );
        cemrTemp.train_eval = EvaluateResults( trainEvalFunc( cemrTemp.regressor ) );
        cemrResults(k,n) = cemrTemp;
        
    end
    
end

resultsName = [trialRoot, 'BeaconCartSim_', num2str(nBeacons), 'Beacons_Results_Splitted_Folded.mat'];

save( resultsName, 'baselineResults', 'mcirResults', 'mcterResults', 'dcirResults', 'dcterResults', ...
    'cemrResults' );