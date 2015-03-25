function [R] = Regress( regressor, f )
% Outputs PD matrices R given a regressor struct and predictors f
% regressor - Regressor model struct specifying the regressor type
% f - Predictor feature vectors

if strcmp( regressor.type, 'modified_cholesky' )
    R = ModifiedCholeskyRegression( f, regressor );
elseif strcmp( regressor.type, 'positive_combination' )
    R = PositiveBasisRegression( f, regressor );
elseif strcmp( regressor.type, 'constant' )
    R = repmat( regressor.R, [1,1,size(f,2)] );
elseif strcmp( regressor.type, 'mle' )
    R = repmat( regressor.R, [1,1,size(f,2)] );
else
    error( 'Invalid regressor type.' );
end
