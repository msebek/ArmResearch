function [x,y,z]=yuanWahba(m)

% generates heteroscedastic data as described in [Yuan, Wahba 2004]
% and plotes the data as well as the generating function
%   m : number of grid-points in (0,1)

% (C) Copyright 2006 by Kristian Kersting 2006-12-12.

x = 1:m;
x=(x-0.5)/m;
mean = 2*(exp(-30*(x-0.25).^2)+sin(pi*x.*x));
var  = exp(2*sin(2*pi*x));
y=normrnd(mean,sqrt(var));

z=sqrt(var);

figure;
plot(x,mean,'r');
hold on
plot(x,mean+2*sqrt(var),'b--');
plot(x,mean-2*sqrt(var),'b--');
plot(x,y,'go');
hold off
grid on
title('Yuan, Wahba [2004] Original Function: mean (r), 2*s (b)')

