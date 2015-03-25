function ll = mlhgplikelihood(params,hetGP)
% LL = mlhgplikelihood(PARAMS,HETGP)
%
% Computes the MAP likelihood of the heteroscedastic GP stored in HETGP.
% Note that the data is stored in HETGP (HETGP.x and HETGP.y). Here PARAMS is a
% row vector storing the hyperparamteres of the noise-free GP, the
% hyperparameters of the noise GP, and the latent training points for the
% the noise GP.
%
% (C) copyright 2007, Kristian Kersting

% global variables storing the noise process
global gpNoise;

% copy the parameters
hetGP.theta3 = params(1:hetGP.numParaCov3)';
hetGP.theta2 = params(hetGP.numParaCov3+1:hetGP.numParaCov3+hetGP.numParaCov2)';
hetGP.z = params(hetGP.numParaCov3+hetGP.numParaCov2+1:end)';

% initialize the noise process
gpNoise.theta = hetGP.theta2;
gpNoise.cov = hetGP.covfunc2;
gpNoise.x = hetGP.xx;
gpNoise.z = hetGP.z;

% information about the domain
[n, D] = size(hetGP.x);

% initialize the log-likelihood
ll=0.0;

% compute the log-likelihood of the primary (noise "free") GP
K = feval(hetGP.covfunc3{:}, hetGP.theta3, hetGP.x); % compute training set covariance matrix
L = chol(K)';                        % cholesky factorization of the covariance
alpha = solve_chol(L',hetGP.y); % solves L'*alpha=y also eigentlich K*alpha=y
ll = ll + 0.5*hetGP.y'*alpha + sum(log(diag(L)))  + 0.5*n*log(2*pi);

% compute the log-likelihood of the noise process
% Note that this is the posterior given the "stuetzstellen" and theta
K = feval(hetGP.covfunc2{:}, hetGP.theta2, gpNoise.x);    % compute training set covariance matrix
L = chol(K)';                        % cholesky factorization of the covariance
alpha = solve_chol(L',gpNoise.z); % solves L'*alpha=y also eigentlich K*alpha=y

[Kss, Kstar] = feval(hetGP.covfunc2{:}, hetGP.theta2, gpNoise.x, gpNoise.x);

% log space behaviour
v = L\Kstar;
varlog = Kss - sum(v.*v)'; %predicted variances

% Note that y'*K^-1*y, where y is the latent noise process, is always zero as y is
% predicted by the latent noise process.

ll2 =  sum(log(varlog)) + .5*n*log(2*pi);

% add both likelihoods
ll = ll+ ll2;






