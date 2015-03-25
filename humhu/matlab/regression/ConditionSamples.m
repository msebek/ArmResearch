function [x] = ConditionSamples( x, xMin )
% Prevents samples from falling below a minimum threshold
% x - vectors of samples x
% xMin - vector of minimum x element values

xCutoff = abs(x) < xMin;
cutoffs = x(xCutoff);

xFloors = xMin*sign( cutoffs );
zers = xFloors( xFloors == 0 );
xFloors( zers ) = sign( rand(sum(zers(:)),1) - 0.5 );

x( xCutoff ) = xFloors;
