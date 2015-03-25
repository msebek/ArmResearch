function [mea,s2]= predmlhgp(hetGP,xstar)
% [MEA,S2]= predmlhgp(HETGP,XSTAR)
%
% Compute predictive mean MEA and predictive covariance S2
% of an heteroscedastic Gaussian Process. Here, HETPG stores the
% heteorscedastic GP as well as the training data and all hyperparameters.
% XSTAR is a column vector storing the test points.
%
% (C) Copyright 2006 by Kristian Kersting, Christian Plagemann 2006-12-12.


global gpNoise;

% set the noise process
gpNoise.cov = hetGP.covfunc2;
gpNoise.theta = hetGP.theta2;
gpNoise.x = hetGP.xx;
gpNoise.z = hetGP.z;


% compute the mean and predictive uncertainty of het GP
[mea, s2] = gpr(hetGP.theta3, hetGP.covfunc3, hetGP.x, hetGP.y, xstar);




    