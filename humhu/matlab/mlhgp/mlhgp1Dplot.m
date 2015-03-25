function [mea, S2,meaGP, S2GP]=mlhgp1Dplot(hetGP,a,b,n)
% [mea, S2,meaGP, S2GP]=mlhgp1Dplot(hetGP,a,b,n)
%
% Plots a one dimensional (R->R) Gaussian Process over the input
% intervall [a:n:b]
%
% Usage: mlhgp1Dplot(hetGP,a,b,n)
%
% where hetGP is the input heteroscedastic GP (which includes the
% data and the hyper-paramters etc.). The predictive mean as well as
% the 2*std confidence intervalls are shown. Next to the heteroscedastic
% process the standard process is plotted as well (i.e., the primary
% process).
%
% (C) Copyright 2007 by Kristian Kersting

xstar = linspace(a,b,n)';
figure

subplot(1,2,2);
[mea S2] = gpr(hetGP.theta1, hetGP.covfunc1, hetGP.x, hetGP.y, xstar);
plot(hetGP.x, hetGP.y, 'go');
hold on;
plot(xstar, mea+2*sqrt(S2),'--b');
plot(xstar,mea,'r');
plot(xstar,mea-2*sqrt(S2),'--b');
grid
axis square
title('Gaussian Process');

meaGP = mea;
S2GP = S2;

[mea, S2] =  mlhgppred(hetGP,xstar);

subplot(1,2,1);
plot(hetGP.x, hetGP.y, 'go');
hold on
plot(xstar, mea+2*sqrt(S2),'--b');
plot(xstar,mea,'r');
plot(xstar,mea-2*sqrt(S2),'--b');
grid
axis square
title('Informative Noise Vector Machine');

