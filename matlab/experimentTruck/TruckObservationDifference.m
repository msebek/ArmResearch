function [zDiff] = TruckObservationDifference( a, b )
zDiff = a - b;
zDiff(2,:) = wrapToPi(zDiff(2,:));