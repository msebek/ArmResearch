%% Example framework for training a covariance model

%% 0. Problem parameters
nDim = 3; % Dimensionality of the matrix to be regressed
fDim = 3; % Dimensionality of the predictor vector to use

vDim = nDim*(nDim-1)/2; % The number of lower triangular terms to regress
wDim = nDim;            % The number of diagonal terms to regress

%% 1. Initialize the model struct
% Types are 'modified_cholesky', 'positive_basis', 'constant'
mcir.type = 'modified_cholesky';

% The additive damping matrix helps condition the output 
mcir.damping = 1E-6*eye(nDim);

% Model modes are 'covariance' for regressing covariance matrices or
% 'information' for regressing information matrices. Information is not
% well-supported right now.
mcir.modelMode = 'covariance';

% Matrix structure can be set to 'dense' for regressing the full
% correlations or 'diagonal' for only the variance elements.
mcir.matrixStructure = 'dense';

% Typically you can initialize the parameters to all zeros to start. This
% returns constant identity matrices from the model. Alternatively you can
% first fit a diagonal model and then refine it here.
mcir.v = zeros( fDim, vDim );
mcir.w = zeros( fDim, wDim );

% These are L2 penalties on the v and w parameters. Use at your own
% judgment.
mcir.vPenalty = 1E-3;
mcir.wPenalty = 1E-3;

% Input predictor vectors are element divided by this vector for numerical
% stability. Set it to be approximately the max magnitude you expect to
% see.
mcir.featureScales = ones(fDim,1);

%% 2. Initialize the training parameter struct
% Types are 'innovation' for filter-based iterative fitting, 'EM' for
% smoother-based iterative fitting, 'error' for direct sample fitting, and
% 'mle' for EM-based fitting of constant matrices.
trainingParams.type = 'innovation';

% Other training types have more parameters, such as EM.

%% 3. Train the model
% The training process needs an objective function. Provide it here as a
% function handle that takes a model struct as an input and returns a
% result struct. See 'experimentBeacon/RunBeaconResults.m' for an example.
% 
% NOTE: This is pretty clunky and will be updated in the future.
trainEvalFunc = [];

% Run the training process
[mcirResults.regressor, mcirResults.train] = TrainRegressor( trainEvalFunc, mcir, trainingParams );