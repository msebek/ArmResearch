% Script to visualize covariance bounds and noise
load '../data/beaconTrials/BeaconCartSim_2Beacons_Results.mat';

trainNames = { '../data/beaconTrials/BeaconCartSim_2Beacons_Trial1.mat' };
trainEvalFunc = @(regressor) RunBeaconResults( trainNames, regressor );

mcirTrace = trainEvalFunc( mcirResults.regressor );
cirTrace = trainEvalFunc( cirResults.regressor );

for i = 1:4
    fh = figure;
    ah = axes( 'parent', fh );
    hold( ah, 'on' );
    plot( ah, mcirTrace.true.errors(i,:), 'b.' );
    plot( ah, sqrt(squeeze(mcirTrace.filter.prioriR( i,i,: ))), 'b-' );
    plot( ah, -sqrt(squeeze(mcirTrace.filter.prioriR( i,i,: ))), 'b-' );
%     plot( ah, sqrt(squeeze(cirTrace.filter.prioriR( i,i,: ))), 'r-' );
%     plot( ah, -sqrt(squeeze(cirTrace.filter.prioriR( i,i,: ))), 'r-' );
    title( ['Noise covariance ', num2str(i)] );
    
    fh = figure;
    ah = axes( 'parent', fh );
    hold( ah, 'on' );
    plot( ah, mcirTrace.filter.innovations(i,:), 'b.' );
    mcirPC = mcirTrace.filter.prioriR + mcirTrace.filter.prioriCovariances;
    plot( ah, sqrt(squeeze( mcirPC(i,i,:) )), 'b-' );
    plot( ah, -sqrt(squeeze( mcirPC(i,i,:) )), 'b-' );
%     plot( ah, sqrt(squeeze( cirPC(i,i,:) )), 'r-' );
%     plot( ah, -sqrt(squeeze( cirPC(i,i,:) )), 'r-' );
    title( ['Innovation covariance ', num2str(i)] );
end
