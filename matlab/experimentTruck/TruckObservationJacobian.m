function [J] = TruckObservationJacobian( x, landmark )

dx = x(1) - landmark(1);
dy = x(2) - landmark(2);
r = norm(x(1:2) - landmark(1:2));

J = [ dx./r,    -dy./( dx.^2 + dy.^2 );
      dy./r,    dx./( dx.^2 + dy.^2 );
      0,        -1 ];
J = J'; % Oops!