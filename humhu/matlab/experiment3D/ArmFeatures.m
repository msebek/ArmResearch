function [f] = ArmFeatures( zPred )
% Predicted observations should be SE3 objects

N = size( zPred, 2 );

v = zPred.GetVector();
fDim = 5;
f = zeros(fDim,N);

for i = 1:N
   range = norm( v(1:3,i) );
   eul = quat2euler( v(4:7,i) );
   f(:,i) = [ range; 
              abs( v(2,i) );
              abs( v(3,i) );
              abs( eul(1) ); 
              abs( eul(2) ) ];
end

