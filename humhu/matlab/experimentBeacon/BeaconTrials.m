%% Load trials and run their filters

% trialRoot = 'C:/Users/Humhu/Dropbox/Research/Papers/rss2015/data/beaconTrialsCross/';
trialRoot = '../data/beaconTrials/';

load( [trialRoot, 'BeaconDatasetParams.mat' ] );

% matlabpool open;

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

numParamSets = numel( testParams ); % Test only 2,3,4,5 since 6,7,8 slow for now
numFolds = 4;

for i = 1:numParamSets
    
    foldSize = testParams(i).nIters/numFolds;
    if rem( testParams(i).nIters, numFolds ) > eps
        error( 'Incorrect number of folds!' );
    end
    folds = repmat( 1:numFolds, foldSize, 1 );
    folds = folds(:)';
    
    for n = 1:numFolds
        foldParams = testParams(i);
        foldParams.testInds = find( folds == n );
        foldParams.trainInds = find( folds ~= n );
        postfix = ['_Fold', num2str(n)];
        RunBeaconTrial( foldParams, trialRoot, postfix );
    end
    
end

% matlabpool close;

%% Test to see performance vs. data

i = 2; % 3 beacons
 allInds = 1:testParams(i).nIters;
 foldSize = testParams(i).nIters/numFolds;
 if rem( testParams(i).nIters, numFolds ) > eps
     error( 'Incorrect number of folds!' );
 end
 folds = repmat( 1:numFolds, foldSize, 1 );
 folds = folds(:)';

for s = 1:3:(numel(allInds) - foldSize)
    parfor n = 1:numFolds
        splitParams(n) = testParams(i);
        splitParams(n).testInds = find( folds == n );
        splitParams(n).trainInds = find( folds ~= n );
        splitParams(n).trainInds = splitParams(n).trainInds(1:s);
        postfix = ['_Split', num2str(s), '_Fold', num2str(n)];
        RunBeaconTrial( splitParams(n), trialRoot, postfix );
    end
end