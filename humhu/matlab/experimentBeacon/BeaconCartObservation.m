function [z] = BeaconCartObservations( cart, beacons )
% Generates observations for T cart positions from the beacons
% Uses vector form for poses to be useful to the EKF

nBeacons = size( beacons, 2 );
T = size( cart, 2 );

zDim = 2*nBeacons;
z = zeros( zDim, T );

for i = 1:nBeacons
    beacon = beacons(:,i);
    
    pos = bsxfun( @minus, beacon(1:2), cart(1:2,:) );
    r = sqrt( sum( pos.*pos, 1 ) );
    b = atan2( beacon(2) - cart(2,:), beacon(1) - cart(1,:) ) - cart(3,:);
    b = wrapToPi(b);
    
    z( (2*(i-1)+1):2*i, : ) = [r; b];
    
end