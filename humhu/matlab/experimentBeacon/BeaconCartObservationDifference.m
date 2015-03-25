function [dz] = BeaconCartObservationDifference( z1, z2 )
dz = z1 - z2;
dz(2:2:end,:) = wrapToPi( dz(2:2:end,:) );