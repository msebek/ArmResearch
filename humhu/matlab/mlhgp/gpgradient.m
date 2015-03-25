function grad = gpgradient(params,covfunc,X,y)
%[GRAD] = gpgradient(PARAMS,COVFUNC,X,y)
%
% Computes the partial derivates of the GP
%
% (C) Copyright 2007, Kristian Kersting.

[ll,grad] = gpr(params', covfunc, X, y);

grad = grad';