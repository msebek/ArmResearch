function [xNew] = TruckTransition( x, H )

ct = cos(x(3));
st = sin(x(3));
Hx = [ ct, -st, x(1);
       st, ct, x(2);
       0, 0, 1 ];

Hnew = Hx*H;
xNew = [ Hnew(1:2,3); atan2( Hnew(2,1), Hnew(1,1) ) ];