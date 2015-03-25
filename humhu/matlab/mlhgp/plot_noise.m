function  h=plot_noise(x,y,z)

h=figure;
plot(x,z,'k');
hold on;
plot(x,y,'*r');
title('original data set (mean k)');