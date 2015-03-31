function [xi theta] = homtotwist(t)
% Copied from the matlab kinematics library
% Credit as assigned there. 
%
%  [XI THETA] = HOMTOTWIST(T)
%
%   Note that there are multiple possible combinations of
%       xi and theta that satisfy exp(xi*theta) = T.
%
%       Theta is thus returned with a rance of 0 to pi.
%
%   TODO verify why this holds. 

  %if ~ishom(t),
  %    error('SCREWS:homtotwist', 'T must be a homogeneous transform');
  %end

  xi = zeros(6, 1);
  
  % Algorithm from Murray pg43
  % TODO who is this? what book is this?
  
  % if it's only translation
  if my_isequalf(rot(t), eye(3)),
      %if (R, p) == (I, 0)
      if my_isequalf(pos(t), zeros(3,1))
          theta = 0;
      else
          theta = norm(pos(t));
          xi(1:3) = pos(t) / theta;
      end
  else
      % If it's a rotation and translation
     [omega, theta] = my_rotparam(rot(t));
     omega_hat = my_skew(omega);
     
     A = (eye(3) - skewexp(omega_hat, theta))*omega_hat ...
         + omega*omega'*theta;
     v = linsolve(A, pos(t));
     
     xi(1:3) = v;
     xi(4:6) = omega;   
  end
  
end