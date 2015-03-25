function [xNew] = BeaconCartTransition( x, u, dt )

x2 = SE2( x, 'vector' );
u = [u(1);0;u(2)];
displacement = SE2( u*dt, 'twist' );
xNew2 = x2*displacement;
xNew = xNew2.GetVector();