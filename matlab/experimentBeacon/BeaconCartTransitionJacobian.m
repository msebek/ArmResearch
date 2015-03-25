function [J] = BeaconCartTransitionJacobian(x,u)

N = size(u,2);
    drotmat = @(th) [ -sin(th), -cos(th);
        cos(th), -sin(th) ];
    
J = zeros(3,3,N);
for i = 1:N
    J(:,:,i) = eye(3) + [ zeros(3,2), [drotmat(x(3))*[u(1,i);0]; 0] ];
end