function [mea,s2]= mlhgppred(hetGP,xstar)

% Compute predictive mean mea and predictive covariance s2
% of an heteroscedastic Gaussian Process hetGP
%
% Usage: [mea], s2] = mlhgppred(hetGP,xstar)
%
% (C) Copyright 2007 by Kristian Kersting.

global gpNoise;

% set the noise process
gpNoise.cov = hetGP.covfunc2;
gpNoise.theta = hetGP.theta2;
gpNoise.x = hetGP.xx;
gpNoise.z = hetGP.z;

% compute the mean and predictive uncertainty of het GP
[mea, s2] = gpr(hetGP.theta3, hetGP.covfunc3, hetGP.x, hetGP.y, xstar);




    