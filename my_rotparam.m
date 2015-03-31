function [omega theta] = my_rotparam(r)
% copied directly from 
%
%   [OMEGA THETA] = MY_ROTPARAM(R)
%
%   pulls a skew matrix and theta out of a rotation matrix

    if ~isrot(r),
        error('SCREWS:rotparam', 'T must be a rotation');
    end

    t = (trace(r) - 1) / 2;

    if t < -1,
        t = 1;
    elseif t > 1,
        t = 1;
    end


    % from Murray, pg 29-30
    theta = acos(t);

    omega = my_rotaxis(r, theta);

end