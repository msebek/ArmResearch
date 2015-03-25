function [C] = randpd(n,s)
% randpd - Generates random positive definite matrices.
% Usage:
% C = randpd(n,s) - Generates a n x n x m matrix C where each C(:,:,i) is
%   a PD matrix. Takes m scales s to generate random square root matrices L
%   with elements Li in [-s,s], then calculates C = L'*L.
%
% Notes:
%   This method tends to produce matrices with very elliptical eigenvalues.
%   Why this is the case is still unclear to me.

M = numel(s);

L = 2*(rand(n,n,M) - 0.5);
C = zeros(n,n,M);
for i = 1:M
   C(:,:,i) = s(i)*L(:,:,i)'*L(:,:,i); 
end