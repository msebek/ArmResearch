function [J] = BeaconCartObservationJacobian( cart, beacons )

nBeacons = size( beacons, 2 );
J = zeros( 3, nBeacons*2 );

for i = 1:nBeacons
    dx = cart(1) - beacons(1,i);
    dy = cart(2) - beacons(2,i);
    r = norm(cart(1:2) - beacons(1:2,i));

J(:, (2*(i-1)+1):2*i) = [ dx./r, -dy./( dx.^2 + dy.^2 );
                          dy./r, dx./( dx.^2 + dy.^2 );
                          0, -1 ];
end
J = J'; % Oops!