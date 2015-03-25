function ll = gplikelihood(params,covfunc,X,y)
% LL = gplikelihood(PARAMS,COVFUNC,X,y)
%
% Computes the likelihood LL of a GP.
%
% (C) copyright 2007, Kristian Kersting


ll = gpr(params', covfunc, X, y);






