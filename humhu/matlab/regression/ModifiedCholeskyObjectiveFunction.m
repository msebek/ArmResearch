function [lval, grad] = ModifiedCholeskyObjectiveFunction( paramVec, model, x, f, C )
% Objective function for use with fminunc. Note that it catches poorly
% conditioned log likelihoods and returns an Inf objective value

zdim = size(model.w,2);
fdim = size(model.w,1);

vdim = (zdim - 1)*zdim/2;
wdim = zdim;

vInds = 1:fdim*vdim;
wInds = vInds(end)+1:(vInds(end)+fdim*wdim);

model.w = reshape( paramVec(wInds), fdim, wdim );
model.v = reshape( paramVec(vInds), fdim, vdim );

S = ModifiedCholeskyRegression( f, model );
S = S + C;

likelihood = GaussianLogLikelihood( x, S, model.modelMode );
lval = likelihood - model.vPenalty*0.5*model.v(:)'*model.v(:) ...
    - model.wPenalty*0.5*model.w(:)'*model.w(:);
lval = -mean(lval); % Negative for minimization

if isinf(lval)
    lval = Inf;
    grad = zeros( size(paramVec) );
    return;
end
    
% x = ConditionSamples( x, 1E-3 );
[dV, dW] = ModifiedCholeskyGradient( x, f, C, model );

if strcmp( model.matrixStructure, 'diagonal' )
    dV = zeros(size(dV));
    grad = -[dV(:); dW(:)]';    
elseif strcmp ( model.matrixStructure, 'dense' )
    grad = -[dV(:); dW(:)]';    
else
    error( 'Invalid matrix structure specification.' );
end

