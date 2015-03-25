function [f] = BeaconCartFeatures( obs )
% Generates features from system observations of interleaved range, bearing
% Note that features should be generated from predicted observations when
%   actually executing a filter, not the true observations!

zDim = size( obs, 1 );
nBeacons = zDim/2;
nObs = size( obs, 2 );

fDim = 3; % Per beacon
f = zeros(fDim*nBeacons + 1, nObs ); 
for i = 1:nBeacons
    range = obs( 2*i-1, : );
%     bearing = obs( 2*i, : );
    
%     f( (fDim*(i-1)+1):fDim*i, : ) = [ range;
%                                       1./range;
%                                       1./(range.*range);
%                                       abs(bearing) ];
    f( (fDim*(i-1)+1):fDim*i, : ) = [ log(range);
                                      range;
                                      1./(range+1) ];
end
f(end,:) = ones(1,nObs);