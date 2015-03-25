function bp=mlhgpstruct(x,y,covfuncGP,covfuncNoise)
% bp=mlhgpstruct(x,y,covfuncGP,covfuncNoise)
%
% Builds the structure used to encode the heteroscedastic Gaussian Process
% The covariance function of the combined GP is the one of the sum of the 
% primary GP plus the noise GP
%
% INPUT
% x : input data
% y : output data
% covfuncGP : complete covariance function of primary GP
% covfuncGPnoise : complete covariance function of noise GP
%
% OUTPUT
% bp.x : input data
% bp.y : output data
% bp.z : empirical noise error (initialized as zero)
% bp.dim : numer of inpout features
% bp.covfunc1 : primiary GP covariance function 
% bp.covfunc2 : noise GP covariance function 
% bp.covfunc3 : combined GP covariance function 
% bp.thetai : hyper-paramteres GP i (randomly initialized)
% bp.numParaCovi : number of hyper-paramteres GP i
%
% (C) Copyright 2006 by Kristian Kersting, Christian Plagemann 2006-12-12.

% input data 
bp.x = x;

% output data
bp.y = y;

% empirical noise residual
bp.z = zeros(size(x));

% dimension of data
bp.dim = size(x,2);

D=bp.dim;

% primary (normal) Gaussian process
bp.covfunc1=covfuncGP;
bp.numParaCov1=eval(feval(bp.covfunc1{:}));
bp.theta1 = rand(bp.numParaCov1,1);

%seconary (noise) Gaussian process
bp.covfunc2=covfuncNoise;
bp.numParaCov2=eval(feval(bp.covfunc2{:})); 
bp.theta2 = rand(bp.numParaCov2,1);

%Ternary GP using the noise process as noise facotr (diagonal matrix
%r(x_i))
bp.covfunc3=covfuncGP;
bp.covfunc3{1,end}{end}='covmlhgp';
bp.covfunc3{1,end}{end+1}='covNoise'; 
bp.numParaCov3=eval(feval(bp.covfunc3{:})); 
bp.theta3 = rand(bp.numParaCov3,1);




