% Assuming [vector, scalar] format for quaternions
function [q] = quatmult(q1, q2)

orientation = size(q1);
q1 = reshape(q1, 4, 1);
q2 = reshape(q2, 4, 1);

s1 = q1(4);
v1 = q1(1:3);

s2 = q2(4);
v2 = q2(1:3);

s = s1*s2 - v1'*v2;
v = s1*v2 + s2*v1 + cross(v1, v2);
q = [v; s];

q = reshape(q, orientation);