function [lval, grad] = PositiveBasisObjectiveFunction( paramVec, params, x, f, C )
% Objective function for use with fminunc

params.w = reshape( paramVec, size(params.w) );
S = PositiveBasisRegression( f, params );
S = S + C;
T = size(S,3);

likelihood = GaussianLogLikelihood( x, S, params.modelMode );
lval = likelihood - params.wPenalty*0.5*params.w(:)'*params.w(:);
lval = -mean(lval); % Negative for minimization

dW = PositiveBasisGradient( x, f, C, params );
grad = -dW(:)';