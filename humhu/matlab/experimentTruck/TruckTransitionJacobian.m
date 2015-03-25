function [J] = TruckTransitionJacobian(x,H)

ct = cos(x(3));
st = sin(x(3));
drotmat = [ -st, -ct;
             ct, -st ];
vel = norm(H(1:2,3));      
temp = drotmat*[vel;0];
J = [ 1, 0, temp(1);
      0, 1, temp(2);
      0, 0, 1 ];
% J = eye(3) + [ zeros(3,2), [drotmat*[vel; 0]; 0] ];