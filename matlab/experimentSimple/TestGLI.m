% Generates a random GLI function, samples from it, and attempts to regress out the parameters
%% Parameters
% Dimensionality parameters
xDim = 3;
fDim = 6;

% GLI function generation parameters
vScale = 1;
vMean = 0;
wScale = 1;
wMean = 0;
fScale = 1;
fMean = 0;

% Data generation parameters
numDatapoints = 1000;
xMean = zeros(xDim,1);

% Gradient ascent parameters
vPenalty = 1E-1;
wPenalty = 0;
ascentParams.stepSize = 1;
ascentParams.backtrackRatio = 0.8;
ascentParams.gradientTolerance = 1E-4; % Stop after average per element gradient norm drops below this
ascentParams.maximumIterations = 500;
ascentParams.maximumBacktracks = 12;
ascentParams.sparsify = false;

vDim = (xDim -1 )*xDim/2; % Don't need terms for diagonal of L
wDim = xDim;

%% Generate random GLI function
vTrue = vScale*rand( fDim, vDim ) - (vScale/2 - vMean);
wTrue = wScale*rand( fDim, wDim ) - (wScale/2 - wMean);

trueParams.v = vTrue;
trueParams.w = wTrue;
trueParams.vPenalty = vPenalty;
trueParams.wPenalty = wPenalty;

%% Initialize guess GLI parameters
vGuess = zeros( fDim, vDim );
wGuess = zeros( fDim, wDim );

initParams.v = vGuess;
initParams.w = wGuess;
initParams.vPenalty = vPenalty;
initParams.wPenalty = wPenalty;

%% Generate random data
f = fScale*rand( fDim-1, numDatapoints ) - (fScale/2 - fMean);
f = [f; 0.1*ones(1, numDatapoints)];

%f = fScale*rand( fDim, numDatapoints ) - (fScale/2 - fMean);

trueInfo = LinearCovarianceModel( f, trueParams );
trueCov = zeros( xDim, xDim, numDatapoints );

maxTrueCov = -Inf;
minTrueCov = Inf;
for i = 1:numDatapoints
   trueCov(:,:,i) = inv(trueInfo(:,:,i)); 
   cVals = eig( trueCov(:,:,i) );
   larger = cVals > maxTrueCov; 
   if any( larger )
       maxTrueCov = cVals(find( larger, 1 ));
   end
   smaller = cVals < minTrueCov;
   if any( smaller )
       minTrueCov = cVals(find( smaller, 1 ));
   end
end

x = mvnrnd( xMean', trueCov )';

%% Perform GLI gradient ascent
estimatedParams = GLILikelihoodGradientAscent( x, f, initParams, ascentParams );
boundParams = GLILikelihoodGradientAscent( x, f, trueParams, ascentParams );

%% Calculate estimated covariances
estInfo = LinearCovarianceModel( f, estimatedParams );
estCov = zeros( xDim, xDim, numDatapoints );
for i = 1:numDatapoints
   estCov(:,:,i) = inv(estInfo(:,:,i)); 
end

maxEstCov = -Inf;
minEstCov = Inf;
for i = 1:numDatapoints
   estCov(:,:,i) = inv(estInfo(:,:,i)); 
   cVals = eig( estCov(:,:,i) );
   larger = cVals > maxEstCov; 
   if any( larger )
       maxEstCov = cVals(find( larger, 1 ));
   end
   smaller = cVals < minEstCov;
   if any( smaller )
       minEstCov = cVals(find( smaller, 1 ));
   end
end

%% Report parameter errors
% Absolute error statistics
vErr = trueParams.v - estimatedParams.v;
wErr = trueParams.w - estimatedParams.w;

meanVErr = mean( abs(vErr(:)) );
maxVErr = max( abs(vErr(:)) );
meanWErr = mean( abs(wErr(:)) );
maxWErr = max( abs(wErr(:)) );

% Relative error statistics
relVErr = vErr./trueParams.v;
relWErr = wErr./trueParams.w;

meanRelVErr = mean( relVErr(:) );
maxRelVErr = max( relVErr(:) );
meanRelWErr = mean( relWErr(:) );
maxRelWErr = max( relWErr(:) );

fprintf( 'Absolute error statistics:\n' );
fprintf( ['\tv error mean: ', num2str(meanVErr), ' max: ', num2str(maxVErr), '\n'] );
fprintf( ['\tw error mean: ', num2str(meanWErr), ' max: ', num2str(maxWErr), '\n'] );
fprintf( 'Relative error statistics:\n' );
fprintf( ['\tv rel error mean: ', num2str(meanRelVErr), ' max: ', num2str(maxRelVErr), '\n'] );
fprintf( ['\tw rel error mean: ', num2str(meanRelWErr), ' max: ', num2str(maxRelWErr), '\n'] );

%% Report likelihood comparison
trueLikelihoods = CalculateGLILikelihood( x, f, trueParams );
estimatedLikelihoods = CalculateGLILikelihood( x, f, estimatedParams );
boundLikelihoods = CalculateGLILikelihood( x, f, boundParams );

meanTrueLikelihood = mean( trueLikelihoods );
meanEstLikelihood = mean( estimatedLikelihoods );
meanBoundLikelihood = mean( boundLikelihoods );

fprintf( 'Likelihood statistics:\n' );
fprintf( ['\ttrue likelihood: ', num2str(meanTrueLikelihood), '\n'] );
fprintf( ['\testimated likelihood: ', num2str(meanEstLikelihood), '\n'] );
fprintf( ['\tbound likelihood: ', num2str(meanBoundLikelihood), '\n'] );

%% Report spectral norm comparison
fprintf( 'Spectral norms:\n' );
fprintf( ['\ttrue cov spectral norm: ', num2str( sqrt(maxTrueCov) ), ...
    ' info spectral norm: ', num2str( sqrt(1/minTrueCov) ), '\n'] );
fprintf( ['\testimated cov spectral norm: ', num2str( sqrt(maxEstCov) ), ...
    ' info spectral norm: ', num2str( sqrt(1/minEstCov) ), '\n'] );
