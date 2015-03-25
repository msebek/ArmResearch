function [aa] = quatlog(q)

q = quatnorm(q);
angle = wrapToPi(2*acos(q(4)));

if angle == 0
   axis = zeros(3,1); 
else
    v = q(1:3);
    axis = v/norm(v);
end
aa = angle*axis;