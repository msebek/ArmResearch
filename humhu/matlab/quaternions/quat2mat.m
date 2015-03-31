% QUAT2MAT - Turns quaternions into a rotation matrix
% Usage:
%  R = quat2mat(q) - q should be in [x; y; z; w] format. 
%     q can also be a matrix of quaternions. 
%     R will be a 3 x 3 x N set of rotation matrices.
%  
function [R] = quat2mat(q)
N = size(q,2);

a = reshape(q(4,:), 1, 1, N);
b = reshape(q(1,:), 1, 1, N);
c = reshape(q(2,:), 1, 1, N);
d = reshape(q(3,:), 1, 1, N);

R = [a.*a + b.*b - c.*c - d.*d, 2.*b.*c - 2.*a.*d, 2.*b.*d + 2.*a.*c;
    2.*b.*c + 2*a.*d, a.*a - b.*b + c.*c - d.*d, 2.*c.*d - 2.*a.*b;
    2.*b.*d - 2.*a.*c, 2.*c.*d + 2.*a.*b, a.*a - b.*b - c.*c + d.*d];