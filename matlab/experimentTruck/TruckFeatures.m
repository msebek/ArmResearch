function [f] = TruckFeatures( obs )

range = obs(1,:);
bearing = obs(2,:);
f = [ log( range );
      range;
      log( abs(bearing) );
      abs(bearing);
      ones(1, size(obs,2) ) ];
