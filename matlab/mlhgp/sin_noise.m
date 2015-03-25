
function [x,y,z,sigma]=sin_noise(a,b,n)

x = linspace(a,b,n)';
z = 2*sin(2*pi*x);
sigma = x + 0.5;
y = normrnd(z,sigma); %+100;