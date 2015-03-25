function [qinv] = quatinv(q)

qinv = q;
qinv(1:3) = -qinv(1:3);