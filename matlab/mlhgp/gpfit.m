function newloghyperGP = gpfit(loghyperGP,covfuncGP,Xtr,ytr)
% GPnew = gpfit()
%
% Performce nonlinear gradient-based optimization for estimating the
% parameters of  GP GP. Returns the estimated GP.
%
% Copyright 2007, Kristian Kersting

% use scaled conjugate gradient
% set the options
options = foptions;
options(1) = 1;
options(9) = 0;
options(14) = 30;

% call scg
loghyperGP = scg('gplikelihood',loghyperGP',options,'gpgradient',...
    covfuncGP,Xtr,ytr);

newloghyperGP = loghyperGP';

% % if you want to use CG, use this
% % use conjugate gradient
%  newloghyperGP = minimize(loghyperGP,'gpr',-50,covfuncGP,Xtr,ytr);
% 