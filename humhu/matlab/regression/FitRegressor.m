function [regressor, out] = FitRegressor( samples, features, additive, regressor, options )
% Fits a regressor to specified data
%   samples - Samples from the distribution to fit
%   features - Corresponding features
%   additive - Corresponding additive term matrices. If empty, will be set to
%       zeros

fDim = size( features, 1 );
zDim = size( samples, 1 );
N = size( samples, 2 );

if N == 0
    out.del = 0;
    return;
end

if isempty( additive )
    additive = zeros(zDim, zDim, N);
elseif size(additive,3) == 1
    additive = repmat( additive, [1,1,N] );
end

%     'DerivativeCheck', 'on', 'FinDiffType', 'central', ...
if nargin < 5
    options = optimoptions( 'fminunc', 'algorithm', 'quasi-newton', 'GradObj','on', ...
        'plotfcns', {@optimplotx, @optimplotfval}, 'tolfun', 0, 'Display', 'off' );
end

if strcmp( regressor.type, 'modified_cholesky' )
    
    wDim = zDim;
    vDim = zDim*(zDim-1)/2;
    
    mcObjectiveFunction = @(p) ModifiedCholeskyObjectiveFunction( p, regressor, ...
        samples, features, additive );
    
    mcInitVec = [regressor.v(:); regressor.w(:)];
    [mcParamVec, fval] = fminunc( mcObjectiveFunction, mcInitVec, options );
    
    init_v = regressor.v;
    init_w = regressor.w;
    regressor.v = reshape( mcParamVec(1:fDim*vDim), fDim, vDim );
    regressor.w = reshape( mcParamVec(fDim*vDim + (1:fDim*wDim)), fDim, wDim );
    
    dV = regressor.v - init_v;
    dW = regressor.w - init_w;
    out.del = norm( [dV(:); dW(:)] )/( numel(regressor.v) + numel(regressor.w) );
    
elseif strcmp( regressor.type, 'positive_combination' )
    
    pbObjectiveFunction = @(p) PositiveBasisObjectiveFunction( p, regressor, ...
        samples, features, additive );
    
    pbInitVec = regressor.w(:);
    [paramVec, fval] = fminunc( pbObjectiveFunction, pbInitVec, options );
    
    init_w = regressor.w;
    regressor.w = reshape( paramVec, size(regressor.w) );
    
    dW = regressor.w - init_w;
    out.del = norm( dW(:) )/numel( regressor.w );
    
elseif strcmp( regressor.type, 'constant' )
    
    fDim = 1;
    features = ones(fDim,N);
    
    wDim = zDim;
    vDim = zDim*(zDim-1)/2;
    
    mcObjectiveFunction = @(p) ModifiedCholeskyObjectiveFunction( p, regressor, ...
        samples, features, additive );
    
    mcInitVec = [regressor.v(:); regressor.w(:)];
    [mcParamVec, fval] = fminunc( mcObjectiveFunction, mcInitVec, options );
    
    init_v = regressor.v;
    init_w = regressor.w;
    regressor.v = reshape( mcParamVec(1:fDim*vDim), fDim, vDim );
    regressor.w = reshape( mcParamVec(fDim*vDim + (1:fDim*wDim)), fDim, wDim );
    regressor.R = ModifiedCholeskyRegression( ones(fDim,1), regressor ) + regressor.damping;
    
    dV = regressor.v - init_v;
    dW = regressor.w - init_w;
    out.del = norm( [dV(:); dW(:)] )/( numel(regressor.v) + numel(regressor.w) );

elseif strcmp( regressor.type, 'mle' )
    
    regressor.init_R = regressor.R;
    regressor.R = cov( samples' );
    out.del = norm( regressor.init_R(:) - regressor.R(:) )/zDim;
    fval = GaussianLogLikelihood( samples, repmat( regressor.R, [1,1,N] ), 'covariance' );
    
end

out.fval = fval;