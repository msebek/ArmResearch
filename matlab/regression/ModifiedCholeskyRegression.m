function [S,L,D,lInds] = ModifiedCholeskyRegression( f, model )
% Outputs modified Cholesky matrices L and D
% f - Predictor vectors
% model - Modified Cholesky regressor model struct

% Scale features
if isempty( f )
   f = double.empty( size(model.featureScales,1), 0 ); 
end
f = bsxfun( @rdivide, f, model.featureScales );

l = model.v'*f;
d = exp( model.w'*f );
n = size(d, 1);
N = size(f,2);
D = zeros(n, n, N);

[L, lInds] = treshape( l, 1 );

% baseInds = 1:n+1:n*n;
% indIncrement = n*n;
% inds = baseInds - indIncrement;

S = zeros(n, n, N);
for i = 1:N
    D(:,:,i) = diag( d(:,i) );
%     inds = inds + indIncrement;
    
    %     L(inds) = 1;
    L(:,:,i) = L(:,:,i) + eye(n);
    S(:,:,i) = L(:,:,i)*D(:,:,i)*L(:,:,i)' + model.damping;
end