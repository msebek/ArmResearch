function hetGPnew = mlhgpfit(hetGP)
% HETGPnew = mlhgpfit(HETGP)
%
% Performce nonlinear gradient-based optimization for estimating the
% parameters of the heteroscedastic GP HETGP. Returns the estimated
% heteroscedastic GP.
%
% Copyright 2007, Kristian Kersting

%row vector of parameters to be optimized
theta = [hetGP.theta3' hetGP.theta2' hetGP.z'];


% use scaled conjugate gradient
% set the options
options =foptions;
options(1) = 1;
options(9) = 0;
options(14) = 30;

% call scg
newtheta = scg('mlhgplikelihood',theta,options,'mlhgpgradient',hetGP);

% retransform the estimated parameters
hetGPnew = hetGP;
hetGPnew.theta3 = newtheta(1:hetGP.numParaCov3)';
hetGPnew.theta2 = newtheta(hetGP.numParaCov3+1:hetGP.numParaCov3+hetGP.numParaCov2)';
hetGPnew.z = newtheta(hetGP.numParaCov3+hetGP.numParaCov2+1:end)';

% % if you want to use CG, use this
% % use conjugate gradient
% newtheta = minimize(theta', 'mlhgpobjective', -100,hetGP);
% hetGPnew = hetGP;
% hetGPnew.theta3 = newtheta(1:hetGP.numParaCov3);
% hetGPnew.theta2 = newtheta(hetGP.numParaCov3+1:hetGP.numParaCov3+hetGP.numParaCov2);
% hetGPnew.z = newtheta(hetGP.numParaCov3+hetGP.numParaCov2+1:end);

