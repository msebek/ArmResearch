% Demo for the joint maximization of the noise-free and the noise GPs, i.e.
% the informative Noise Vector Machine
% 
% Copyright 2007 by Kristian Kersting


%Clean up
close all;
clear global gpNoise;
clear;

% set some global variables
global gpNoise;
global noiseFig;

% generate the data
disp('generating the data ...');

%%% uncomment the data set you are interested in

%%%%%%%%%%%%%
% Silverman
%%%%%%%%%%%%%

a=0;
b=60;
data = [  2.4    0.0 ;  2.6   -1.3 ;  3.2   -2.7 ;  3.6    0.0 ;
          4.0   -2.7 ;  6.2   -2.7 ;  6.6   -2.7 ;  6.8   -1.3 ;
          7.8   -2.7 ;  8.2   -2.7 ;  8.8   -1.3 ;  8.8   -2.7 ;
          9.6   -2.7 ; 10.0   -2.7 ; 10.2   -5.4 ; 10.6   -2.7 ;
         11.0   -5.4 ; 11.4    0.0 ; 13.2   -2.7 ; 13.6   -2.7 ;
         13.8    0.0 ; 14.6  -13.3 ; 14.6   -5.4 ; 14.6   -5.4 ;
         14.6   -9.3 ; 14.6  -16.0 ; 14.6  -22.8 ; 14.8   -2.7 ;
         15.4  -22.8 ; 15.4  -32.1 ; 15.4  -53.5 ; 15.4  -54.9 ;
         15.6  -40.2 ; 15.6  -21.5 ; 15.8  -21.5 ; 15.8  -50.8 ;
         16.0  -42.9 ; 16.0  -26.8 ; 16.2  -21.5 ; 16.2  -50.8 ;
         16.2  -61.7 ; 16.4   -5.4 ; 16.4  -80.4 ; 16.6  -59.0 ;
         16.8  -71.0 ; 16.8  -91.1 ; 16.8  -77.7 ; 17.6  -37.5 ;
         17.6  -85.6 ; 17.6 -123.1 ; 17.6 -101.9 ; 17.8  -99.1 ;
         17.8 -104.4 ; 18.6 -112.5 ; 18.6  -50.8 ; 19.2 -123.1 ;
         19.4  -85.6 ; 19.4  -72.3 ; 19.6 -127.2 ; 20.2 -123.1 ;
         20.4 -117.9 ; 21.2 -134.0 ; 21.4 -101.9 ; 21.8 -108.4 ;
         22.0 -123.1 ; 23.2 -123.1 ; 23.4 -128.5 ; 24.0 -112.5 ;
         24.2  -95.1 ; 24.2  -81.8 ; 24.6  -53.5 ; 25.0  -64.4 ;
         25.0  -57.6 ; 25.4  -72.3 ; 25.4  -44.3 ; 25.6  -26.8 ;
         26.0   -5.4 ; 26.2 -107.1 ; 26.2  -21.5 ; 26.4  -65.6 ;
         27.0  -16.0 ; 27.2  -45.6 ; 27.2  -24.2 ; 27.2    9.5 ;
         27.6    4.0 ; 28.2   12.0 ; 28.4  -21.5 ; 28.4   37.5 ;
         28.6   46.9 ; 29.4  -17.4 ; 30.2   36.2 ; 31.0   75.0 ;
         31.2    8.1 ; 32.0   54.9 ; 32.0   48.2 ; 32.8   46.9 ;
         33.4   16.0 ; 33.8   45.6 ; 34.4    1.3 ; 34.8   75.0 ;
         35.2  -16.0 ; 35.2  -54.9 ; 35.4   69.6 ; 35.6   34.8 ;
         35.6   32.1 ; 36.2  -37.5 ; 36.2   22.8 ; 38.0   46.9 ;
         38.0   10.7 ; 39.2    5.4 ; 39.4   -1.3 ; 40.0  -21.5 ;
         40.4  -13.3 ; 41.6   30.8 ; 41.6  -10.7 ; 42.4   29.4 ;
         42.8    0.0 ; 42.8  -10.7 ; 43.0   14.7 ; 44.0   -1.3 ;
         44.4    0.0 ; 45.0   10.7 ; 46.6   10.7 ; 47.8  -26.8 ;
         47.8  -14.7 ; 48.8  -13.3 ; 50.6    0.0 ; 52.0   10.7 ;
         53.2  -14.7 ; 55.0   -2.7 ; 55.0   10.7 ; 55.4   -2.7 ;
         57.6   10.7 ];  

x      = data(:,1);
y      = data(:,2);

Xtr = x;
ytr = y;
N = size(y,1);
 
% figure,
% plot(Xtr,y,'ob');


%%%%%%%%%%%%%%%%%%%%%%%
% Yuan, Wahba
%%%%%%%%%%%%%%%%%%%%%%%

% a = 0;
% b = 1;
% N = 1000;
% [x,y,sig]=yuanWahba(N);
% drawnow;
% Xtr = x';
% ytr = y';

%%%%%%%%%%%%%%%%%%%%%%%
% Goldberg et al.
%%%%%%%%%%%%%%%%%%%%%%%

% a=0;
% b=1;
% N = 100;
% [x,y,z,sig] = sin_noise(a,b,N);
% plot_noise(x,y,z);
% drawnow;
% Xtr = x;
% ytr = y;

%%%%%%%%%%%%%%%%%%%%%%%
% Williams
%%%%%%%%%%%%%%%%%%%%%%%

% a=0;
% b=pi;
% N=100;
% [X,y,z,sig] = williams_noise(a,b,N);
%  figure
%  plot(X,y,'bo'); grid on;hold on
%  drawnow;
% Xtr = X;
% ytr = y;

%%% GP
disp('estimating Gaussian Process for initialization ...');

% covariance of the 'noise-free' GP
covfuncGP  = {'covSum', {'covSEiso','covNoise'}};

% % log hyperparameters for Silverman
%loghyperGP = [2.0; 5.0; 1.0];

% in all other cases uses these initial hyperparameters
loghyperGP = [log(1.0); log(1.0); log(0.01)];

% xDim = 7;
% loghyperGP = log( ones(xDim+1,1) );

%% estimate hyperparameters
loghyperGP = gpfit(loghyperGP,covfuncGP,Xtr,ytr);


%plot  estimated GP
% xstar = linspace(a,b,100)';
xstar = Xtr + eps;
[mea S2] = gpr(loghyperGP, covfuncGP, Xtr, ytr, xstar);
plot(Xtr(:,1), ytr, 'go');
hold on;
plot(xstar(:,1), mea+2*sqrt(S2),'--b');
plot(xstar(:,1),mea,'r');
plot(xstar(:,1),mea-2*sqrt(S2),'--b');
grid
axis square
title('Gaussian Process');
drawnow


%% estimating hetGP
disp('estimating heteroscedastic Gaussian Process ...');

%covariance of the noise GP
covfuncNoise  = {'covSum', {'covSEiso','covNoise'}};

% its hyperparameters
loghyperNoise = [loghyperGP(1); loghyperGP(2); log(0.1)];

% construct the hetGP structure
mlhgp=mlhgpstruct(Xtr,ytr,covfuncGP,covfuncNoise);

%%% initialize the mlhgp parameters
mlhgp.theta1 = loghyperGP;  

% theta2 is the noise process
mlhgp.theta2 = loghyperNoise;

% theta3 is the noiseless GP
% mlhgp.theta3(1:2) = loghyperGP(1:2);
mlhgp.theta3 = loghyperGP;
% mlhgp.theta3(end) = log(0.5);

% how many latent noise variables do we assume
n = size(xstar,1);

% initialize the latent noise ratios.
mlhgp.z = mlhgp.theta1(end)*ones(n,1);

% the noise x positions (linearly spaces)
%mlhgp.xx = linspace(a,b,n)';
mlhgp.xx = Xtr;


% initi the noise process
gpNoise.x = mlhgp.x;
gpNoise.z = mlhgp.z;
gpNoise.zz = mlhgp.z;
gpNoise.cov = mlhgp.covfunc2;
gpNoise.theta = mlhgp.theta2;

% estimate the parameters
mlhgpnew = mlhgpfit(mlhgp);


%% plot  estimated GP
fprintf( 'Estimating GP values...\n' );
% xstar = linspace(a,b,100)';
[mea S2] = mlhgppred(mlhgpnew,xstar);
fprintf( 'Estimation complete\n' );
figure
plot(Xtr(:,1), ytr, 'go');
hold on;
plot(xstar(:,1), mea+2*sqrt(S2),'--b');
plot(xstar(:,1),mea,'r');
plot(xstar(:,1),mea-2*sqrt(S2),'--b');
grid
axis square
title('Heteroscedastic Gaussian Process');
drawnow

