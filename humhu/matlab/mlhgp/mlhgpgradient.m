function grad = mlhgpgradient(params,hetGP)
%[GRAD] = mlhgpgradient(PARAMS,HETGP)
%
% Computes the partial derivates of the joint MAP w.r.t. to the hyperparameters of
% both the noise-free GP and the noise GP stored in HETGP. Here PARAMS is a
% row vector storing the hyperparamteres of the noise-free GP, the
% hyperparameters of the noise GP, and the latent training points for the
% the noise GP. GRAD is the gradient w.r.t. these parameters of the MAP
% likelihood
%
% (C) Copyright 2007, Kristian Kersting.

% Derivation of the gradient:
%
% Let z be the noise levels in log-space and Z = exp(z).
% We seek to find:
% dL / dtheta3   gradient w.r.t. to the hyperparamteres of the primary process
% dL / dz*       gradient w.r.t. to the latent log noise ratios
% dL / dtheta2   gradient w.r.t to the hyperparamteres of the noise process
%
% dL / dtheta3: is the standard stuff, i.e.,
% dL / d. = dL / dz * dL / d.
% 
% The other partial derivatives can be computed as follows
% dL / dz = (dLP(Y|Z) + LP(z|t)) / dz 
%         = dLP(Y|Z) / dz + LP(z|t) / dz
%         = dLP(Y|Z) / dZ * dZ / dz  + LP(z|t) / dz 
%
% Note that we assume that the overall process is
% given as: 
%            P(y|t) = P(y|exp(~z),t,theta3) 
%       with     ~z = arg max_z P(z|z*,t,theta2)
% where z* are the latent noise levels and theta2 are the 
% hyperparamteres of the noise process. That is , we place
% a Gaussian process prior on the log noise levels.  We consider
% the log noise levels for two reasones. First, it guarantees that
% the noise levels are positive. Second, the maximiations is unconstraint.
% without the log space treatment, the problem would be to estimate
% truncated Gaussian distributions. 

% global variable storing the noise level
global gpNoise;


% copy the parameters
hetGP.theta3 = params(1:hetGP.numParaCov3)';
hetGP.theta2 = params(hetGP.numParaCov3+1:hetGP.numParaCov3+hetGP.numParaCov2)';
hetGP.z = params(hetGP.numParaCov3+hetGP.numParaCov2+1:end)';

% information abou the data
n = size(hetGP.x,1);

% initialize the noise process
gpNoise.theta = hetGP.theta2;
gpNoise.x = hetGP.xx;
gpNoise.z = hetGP.z;

%initialize the gradients
gXobs = zeros(size(hetGP.x));
gY = zeros(size(hetGP.theta3));
gX = zeros(size(hetGP.theta2));

% We will now go throught  the gradient computation step by step. 
%

% gradient w.r.t. to the noise "free" gp
% we can use the orinigal implementations
%[ll,gY] = gpr(hetGP.theta3, hetGP.covfunc3, hetGP.x, hetGP.y);

K = feval(hetGP.covfunc3{:}, hetGP.theta3, hetGP.x);    % compute training set covariance matrix
% as a by-product, the noise levels exp(z) respectivel z
% are computed and stored in the glocal structure gpNoise

L = chol(K)';                        % cholesky factorization of the covariance
alpha = solve_chol(L',hetGP.y); % solves L'*alpha=y also eigentlich K*alpha=y
W = L'\(L\eye(n))-alpha*alpha';                % precompute for convenience
for i = 1:length(gY)
    gY(i) = sum(sum(W.*feval(hetGP.covfunc3{:}, hetGP.theta3, hetGP.x, i)))/2;
end



% dL / dz = (dLP(Y|exp(z),t) + LP(z|,z*,t)) / dx = dLP(Y|exp(z),t) / dexp(z) 
%            * d(exp(z)/ dz + LP(z|t) / dX
%

% dLP(Y|Z) / dZ
Kobs = feval(hetGP.covfunc3{:}, hetGP.theta3, hetGP.x);
L = chol(Kobs)';                        % cholesky factorization of the covariance
alpha = solve_chol(L',(hetGP.y)); % solves L'*alpha=y also eigentlich K*alpha=y
W = (alpha*alpha' - L'\(L\eye(n))); %ll

% compute trace over derivatives, ie.
%   dLP(Y|Z) / dZ = diag(W)/2 = 0.5 * trace((alpha*alpha'-invK)*dZ/dz)
 
for i = 1:length(gXobs),
 S = zeros(size(gpNoise.expMean,1));
 S(i,i)=1;
   gXobs(i) = sum(sum(W.*S))/2;
end
  
% dLP(Y|Z) / dz = dLP(Y|Z) / dZ * dZ / z
gXobs = gXobs'*diag(gpNoise.expMean);
 

% dLP(z|z*,t)/dz
% This is indeed 0, i.e., gLx = gXobs

% derivatives w.r.t. z (the latent "stuetzstellen") and the hyperparamters. 
% Note that this is in log space and is not the derivative of
% the log (posterior) likelihood but the realy predicated log noise levels
% z

K = feval(hetGP.covfunc2{:},  hetGP.theta2, gpNoise.x);
L = chol(K)';                   % cholesky factorization of the covariance

% compute (K+sigma)^-1 * y
alpha = solve_chol(L',gpNoise.x); % solves L'*alpha=hetGP.z, i.e., alpha = invK*hetGP.z

[Kss,Kstar] = feval(hetGP.covfunc2{:}, hetGP.theta2, gpNoise.x, hetGP.x);

% dz\dz* where z* are the latent "training" log noise levels we keep
% originally beta = Kstar'* invK, but this should be 
% faster and more stable using the cholesky factorization
invK = inv(K);
beta = (Kstar'*invK);
dxstar = sum(gXobs * beta,1); % Kstar does the smoothing 

% dz\dTh = dz/dK * dK/dTh 
alpha =invK*gpNoise.z;

for i = 1: length(hetGP.theta2)-1,
    derivKstar = feval(hetGP.covfunc2{:}, hetGP.theta2, gpNoise.x, hetGP.x, i);
    derivK     = feval(hetGP.covfunc2{:}, hetGP.theta2, gpNoise.x, i);
    gX(i)  = sum(gXobs*(derivKstar'-beta*derivK)*alpha,1);
end
i = i+1;
derivK     = feval(hetGP.covfunc2{:}, hetGP.theta2, gpNoise.x, i);
gX(i)      = -sum(gXobs*beta*derivK*alpha,1);
    


%put all partial derivatives together
grad = [gY;-gX;-dxstar']';
