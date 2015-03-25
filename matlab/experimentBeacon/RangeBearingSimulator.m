function [noiselessObservations, noisedObservations, noiseValues] = ...
    RangeBearingSimulator( cartPoses, beacons, noiseParams )
% Simulates range-bearing observations for specified observer poses,
% beacon landmark positions, and noise parameters.
%
% Range noise is multiplicative and generated as:
%   s(i) = rangeBiasSample + rangeVarianceSample(i)
%   rangeObs(i) = trueRange(i)*s(i)
%   
%   where rangeBiasSample and rangeVarianceSample are drawn from uniform
%   distributions.
%
% Bearing noise is Gaussian additive:
%   n(i) = normrnd(0, variance)/( range(i) + 1 )
%   bearingObs(i) = trueBearing(i) + n(i)
%
% cartPoses - Array of T SE2 pose objects
% beacons - 2xN matrix of beacon [x;y] locations
% noiseParams - A struct with the fields:
%   rangeBias - [min,max] for the correlated range bias term
%   rangeVariance - [min,max] for the uncorrelated range variance term
%   bearingVariance - Variance for the bearing additive term
%
% trueObs - 2NxT matrix of  [range(1); bearing(1); range(2); ... ]
%           observations without noise
% noisedObs - 2NxT matrix of observations with noise
% noiseValues - The noise values themselves

% Sample multiplicative noise for ranges and Gaussian noise for bearings

T = numel( cartPoses );
N = size( beacons, 2 );
% TODO Input arg checking

% Generate the noiseless observations
noiselessObservations = BeaconCartObservation( cartPoses.GetVector(), beacons );

% Generate range noises
biasScale = noiseParams.rangeBias(2) - noiseParams.rangeBias(1);
rangeBias = biasScale*rand(1,T) + noiseParams.rangeBias(1); 

varyScale = noiseParams.rangeVariance(2) - noiseParams.rangeVariance(1);
rangeVariance = varyScale*rand(N,T) + noiseParams.rangeVariance(1);

rangeNoise = bsxfun( @plus, rangeVariance, rangeBias );

% Generate bearing noises
bearingNoise = mvnrnd( zeros(1,N), 1E-1*eye(N), T )';
bearingNoise = bsxfun( @rdivide, bearingNoise, noiselessObservations(1:2:end,:) + 1 );

% Use noise to generate actual observations
noisedObservations = noiselessObservations;
for i = 1:N
   noisedObservations(2*(i-1)+1,:) =  noisedObservations(2*(i-1)+1,:).*rangeNoise(i,:);
   noisedObservations(2*i,:) = wrapToPi( noisedObservations(2*i,:) + bearingNoise(i,:) );
end
noiseValues = noisedObservations - noiselessObservations;
noiseValues(2:2:end,:) = wrapToPi( noiseValues(2:2:end,:) );