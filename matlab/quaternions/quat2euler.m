%QUAT2EULER - Converts quaternion into an euler angle (3-2-1 convention)
% Usage:
%   eul = quat2euler(q) - q is [x; y; z; w] convention. eul is [psi; theta;
%   phi] output.
function [eul] = quat2euler(q, lock_tol)

if nargin < 2
    lock_tol = 0.499;
end

N = size(q, 2);
x = q(1,:);
y = q(2,:);
z = q(3,:);
w = q(4,:);
temp1 = w.*y - z.*x;

top_lock = temp1 > lock_tol;
bot_lock = temp1 < -lock_tol;
no_lock = ~top_lock & ~bot_lock;

psi = zeros(1, N);
theta = zeros(1, N);
phi = zeros(1, N);

psi(top_lock) = 2*atan2(w(top_lock), x(top_lock));
theta(top_lock) = -pi/2;
phi(top_lock) = 0;

psi(bot_lock) = -2*atan2(x(bot_lock), x(bot_lock));
theta(bot_lock) = pi/2;
phi(bot_lock) = 0;

xnl = x(no_lock);
ynl = y(no_lock);
znl = z(no_lock);
wnl = w(no_lock);
if ~isreal( 2*(wnl.*znl + xnl.*ynl) ) || ~isreal( 1 - 2*(ynl.*ynl + znl.*znl) )
   blah = 1; 
end
psi(no_lock) = atan2( 2*(wnl.*znl + xnl.*ynl), 1 - 2*(ynl.*ynl + znl.*znl) );
phi(no_lock) = atan2( 2*(wnl.*xnl + ynl.*znl), 1 - 2*(xnl.*xnl + ynl.*ynl) );
theta(no_lock) = asin( 2*(wnl.*ynl - znl.*xnl) );

% if(temp1 > 0.499)
%     psi = 2*atan2(q(4,:), q(1,:));
%     theta = -pi/2;
%     phi = psi;
% elseif(temp1 < -0.499)
%     psi = -2*atan2(q(4,:), q(1,:));
%     theta = pi/2;
%     phi = psi;
% else
%     psi = atan2( 2*(q(4,:).*q(3,:) + q(1,:).*q(2,:)), 1 - 2*(q(2,:).*q(2,:) + q(3,:).*q(3,:)) );
%     phi = atan2( 2*(q(4,:).*q(1,:) + q(2,:).*q(3,:)), 1 - 2*(q(1,:).*q(1,:) + q(2,:).*q(2,:)) );
%     theta = asin( 2*(q(4,:).*q(2,:) - q(3,:).*q(1,:)) );
% end

eul = [psi; theta; phi];