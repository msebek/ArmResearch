function [S, w] = PositiveBasisRegression( f, model )
% Regresses PD matrix S using a positive basis model
% f - Predictor vectors
% model - A positive basis regressor struct

% Scale features
f = bsxfun( @rdivide, f, model.featureScales );

nData = size(f,2);
nBasis = size( model.basis, 3 );

w = exp( model.w'*f );
wM = reshape( w, 1, 1, nBasis, nData );

temp = bsxfun( @times, wM, model.basis );
S = squeeze(sum(temp, 3));
S = bsxfun(@plus, S, model.damping );