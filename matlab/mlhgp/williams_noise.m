
function [x,y,m,var]=william_noise(a,b,n)

x = linspace(a,b,n)';

m = sin(2.5*x).*sin((3/2)*x);
var = 1/100 + 1/4 * (1-sin(2.5*x)).^2;

y = normrnd(m,sqrt(var));
var = std(var);