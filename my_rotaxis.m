function axis = my_rotaxis(R, theta)
% MY_ROTAXIS calculates the axis of rotation for a matrix R
%
%   OMEGA = ROTAXIS(R, THETA)
%
%   Given a rotation matrix R and a known THETA, this function
%   returns an axis OMEGA. THETA is limited to a range between 0
%   and pi. There can be potentially be multiple OMEGA values
%   that are correct, so this function will return an 
%   arbitrary one of those

% TODO what does a matrix's axis of rotation mean?

    %if ~isrot(R)
    %    error('SCREWS:rotaxis', 'R must be a rotation matrix');
    %end

    if theta < 0 || theta > pi,
        error('SCREWS:rotaxis', 'theta must be between 0 and pi');
    end

    if my_isequalf(pi, theta) || my_isequalf(0, theta),
        axis = null(R - eye(3));
        axis = axis(:,1) / norm(axis(:,1));
    else
        axis = [R(3,2) - R(2,3); ...
                R(1,3) - R(3,1); ...
                R(2,1) - R(1,2)];
        axis = axis / (2*sin(theta));
    end
end