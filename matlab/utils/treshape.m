function [M, trilInds] = treshape(v,k)
% treshape - Reshapes vectors into lower left triangular matrices.
% Usage:
% M = treshape(v) - Generates n x n x m matrix M where each M(:,:,i) is a
%   lower left triangular matrix using the values of v(:,i). The values of
%   v are mapped as:
%       M(:,:,i) = [ v(1,i), 0, 0, ... 0;
%                    v(2,i), v(3,i), ... 0;
%                    ... ];
% M = treshape(v,k) - Generates n+k x n+k x m matrix M where each M(:,:,i)
%   is a lower left triangular matrix k away from the diagonal.

vn = size(v,1);
m = size(v,2);
n = (sqrt( 1 + 8*vn ) - 1)/2 + k;

if n ~= round(n)
   error( 'v does not have valid number of elements' );
end

%M = zeros(n,n,m);

indM = reshape( 1:(n*n), n, n );
trilInds = tril(indM, -k);
trilInds = trilInds( trilInds ~= 0 );

block = zeros(n*n,m);
block(trilInds,:) = v;

M = reshape(block, n, n, m);

% for i = 1:m
%     temp = zeros(n,n);
%     temp(trilInds) = v(:,i);
% %     M(:,:,i) = reshape(block(:,i),n,n);
%     M(:,:,i) = temp;
% end

