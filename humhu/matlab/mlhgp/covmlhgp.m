function [A, B] = covmlhgp(logtheta, x, z);
% [A, B] = covmlhgp(LOGTHETA, X, Z);
%
% Independent covariance function represented as Gaussian Process.
% The process is assumed to map into the log-space to garantuee that
% all ouput (the noise levels) are positive. For more informations
% on covariance functions call 'help covfunctions'. Note that LOGTHETA is
% actually obsolete as the parameters are stored in the global variable
% gpNoise. For compatibility reasonons, however, we assume they are input.
%
% (C) Copyright 2006 by Kristian Kersting, Christian Plagemann 2006-12-12.

global gpNoise %stores the global noise Gaussian process

if nargin == 0, A = '0'; return; end              
%zero parameters as we have an alternating, separated learning process

if nargin == 2                                      % compute covariance matrix
    [mea,var] = gpr(gpNoise.theta, gpNoise.cov, gpNoise.x,gpNoise.z,x);
    gpNoise.mean = mea;
    gpNoise.var = var;
    s2 = exp(mea);
    A = diag(s2);
    gpNoise.expMean = s2;
elseif nargout == 2                              % compute test set covariances
    [mea,var] = gpr(gpNoise.theta, gpNoise.cov, gpNoise.x,gpNoise.z,z);
    s2 = exp(mea);
    A = s2;
    gpNoise.mean = mea;
    gpNoise.var = var;
    gpNoise.expMean = s2;
    B = 0;                               % zeros cross covariance by independence
else                                                % compute derivative matrix
    A = zeros(size(x,1));
end
