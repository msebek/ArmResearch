%% Load trials and run their filters

dataName = '../data/truck/FoldedData.mat';

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

numFolds = 4;

temp = 1:numFolds;
for n = 1:numFolds
    testParams(n).testInds = n;    
    testParams(n).trainInds = temp( temp ~= n );
end

for n = 1:numFolds
    resultsName = ['../data/truck/Results_Fold', num2str(n), '.mat'];
    RunTruckTrial( testParams(n), dataName, resultsName );
end
