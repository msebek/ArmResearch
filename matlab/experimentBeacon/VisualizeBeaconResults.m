% Plot results from aggregated evaluations over parameter sets. Generates plots of:
%   Filter error (FME)
%   Filter log likelihood (FLL)
%   Innovation log likelihood (ZLL)
%   Smoother error (SME)
%   Smoother log likelihood (SLL)
%   True noise log likelihood (NLL)
% versus the parameter set number

%% Fold processing
% trialRoot = 'C:/Users/Humhu/Dropbox/Research/Papers/rss2015/data/beaconTrials/';
trialRoot = '../data/beaconTrials/';
load( [trialRoot, 'BeaconDatasetParams.mat' ] );

testParams = testParams(1:4);
numParamSets = numel( testParams );
beacons = [testParams.nBeacons];

methodNames = { 'MC', 'MC-T', 'DC', 'DC-T', 'CEM', 'AKF' };
methods = { 'mcir', 'mcter', 'dcir', 'dcter', 'cemr', 'baseline' };
% methodNames = { 'MC', 'MCT', 'CEM', 'AKF' };
% methods = { 'mcir', 'mcter', 'cemr', 'baseline' };
plotStyles = {  'b.-', 'b.--', 'g.-', 'g.--', 'k.-', 'r.-'};
numMethods = numel( methods );

numFolds = 4;

zllTrainingTraces = cell( numMethods, numParamSets, numFolds );
nllTrainingTraces = cell( numMethods, numParamSets, numFolds );
fllTrainingTraces = cell( numMethods, numParamSets, numFolds );

fmeTrain = zeros( numMethods, numParamSets, numFolds );
fllTrain = zeros( numMethods, numParamSets, numFolds );
zllTrain = zeros( numMethods, numParamSets, numFolds );
% smeTrain = zeros( numMethods, numParamSets, numFolds );
% sllTrain = zeros( numMethods, numParamSets, numFolds );
nllTrain = zeros( numMethods, numParamSets, numFolds );

fmeTest = zeros( numMethods, numParamSets, numFolds );
fllTest = zeros( numMethods, numParamSets, numFolds );
zllTest = zeros( numMethods, numParamSets, numFolds );
% smeTest = zeros( numMethods, numParamSets, numFolds );
% sllTest = zeros( numMethods, numParamSets, numFolds );
nllTest = zeros( numMethods, numParamSets, numFolds );

for i = 1:numParamSets
    
    nBeacons = testParams(i).nBeacons;
    nIters = testParams(i).nIters;
    
    resultsName = [trialRoot, 'BeaconCartSim_', num2str(nBeacons), 'Beacons_Results_Folded.mat'];
    fprintf( ['Parsing results from ', resultsName, '\n'] );
    res = load( resultsName );
    for n = 1:numFolds
        for j = 1:numMethods-1
            
            
            methodTest = ['res.', methods{j}, 'Results(', num2str(n), ').test_eval'];
            methodTrain = ['res.', methods{j}, 'Results(', num2str(n), ').train_eval'];
            
            fmeTest(j,i,n) = eval( [methodTest, '.filter.x_avg_err'] );
            fllTest(j,i,n) = eval( [methodTest, '.filter.x_avg_ll'] );
            zllTest(j,i,n) = eval( [methodTest, '.filter.z_avg_ll'] );
            %             smeTest(j,i,k) = eval( [methodTest, '.smoother.x_avg_err'] );
            %             sllTest(j,i,k) = eval( [methodTest, '.smoother.x_avg_ll'] );
            nllTest(j,i,n) = eval( [methodTest, '.true.n_avg_ll'] );
            
            zTemp = eval( ['[res.', methods{j}, 'Results(', num2str(n), ').train.evals.filter]'] );
            nTemp = eval( ['[res.', methods{j}, 'Results(', num2str(n), ').train.evals.true]'] );
            zllTrainingTraces{j,i,n} = [zTemp.z_avg_ll];
            nllTrainingTraces{j,i,n} = [nTemp.n_avg_ll];
            fllTrainingTraces{j,i,n} = [zTemp.x_avg_ll];
            fmeTrainingTraces{j,i,n} = [zTemp.x_avg_err];
            
            fmeTrain(j,i,n) = eval( [methodTrain, '.filter.x_avg_err'] );
            fllTrain(j,i,n) = eval( [methodTrain, '.filter.x_avg_ll'] );
            zllTrain(j,i,n) = eval( [methodTrain, '.filter.z_avg_ll'] );
            %             smeTrain(j,i,k) = eval( [methodTrain, '.smoother.x_avg_err'] );
            %             sllTrain(j,i,k) = eval( [methodTrain, '.smoother.x_avg_ll'] );
            nllTrain(j,i,n) = eval( [methodTrain, '.true.n_avg_ll'] );
            
            
        end
        fmeTest(end,i,n) = res.baselineResults(n).test_eval.adaptive.x_avg_err;
        fllTest(end,i,n) = res.baselineResults(n).test_eval.adaptive.x_avg_ll;
        zllTest(end,i,n) = res.baselineResults(n).test_eval.adaptive.z_avg_ll;
        nllTest(end,i,n) = res.baselineResults(n).test_eval.adaptive.n_avg_ll;
        
        fmeTrain(end,i,n) = res.baselineResults(n).train_eval.adaptive.x_avg_err;
        fllTrain(end,i,n) = res.baselineResults(n).train_eval.adaptive.x_avg_ll;
        zllTrain(end,i,n) = res.baselineResults(n).train_eval.adaptive.z_avg_ll;
        nllTrain(end,i,n) = res.baselineResults(n).train_eval.adaptive.n_avg_ll;
    end
end

fmeTestFolded = mean( fmeTest, 3 );
fllTestFolded = mean( fllTest, 3 );
zllTestFolded = mean( zllTest, 3 );
nllTestFolded = mean( nllTest, 3 );

fmeTrainFolded = mean( fmeTrain, 3 );
fllTrainFolded = mean( fllTrain, 3 );
zllTrainFolded = mean( zllTrain, 3 );
nllTrainFolded = mean( nllTrain, 3 );

%% T-refinement gains for MC and DC
mcInd = find( strcmp( methods, 'mcir' ) );
mctInd = find( strcmp( methods, 'mcter' ) );
dcInd = find( strcmp( methods, 'dcir' ) );
dctInd = find( strcmp( methods, 'dcter' ) );

fmeTestRefinementGain(1,:,:) = fmeTest( mctInd, :, : ) - fmeTest( mcInd, :, : );
fllTestRefinementGain(1,:,:) = fllTest( mctInd, :, : ) - fllTest( mcInd, :, : );
zllTestRefinementGain(1,:,:) = zllTest( mctInd, :, : ) - zllTest( mcInd, :, : );
nllTestRefinementGain(1,:,:) = nllTest( mctInd, :, : ) - nllTest( mcInd, :, : );

fmeTrainRefinementGain(1,:,:) = fmeTrain( mctInd, :, : ) - fmeTrain( mcInd, :, : );
fllTrainRefinementGain(1,:,:) = fllTrain( mctInd, :, : ) - fllTrain( mcInd, :, : );
zllTrainRefinementGain(1,:,:) = zllTrain( mctInd, :, : ) - zllTrain( mcInd, :, : );
nllTrainRefinementGain(1,:,:) = nllTrain( mctInd, :, : ) - nllTrain( mcInd, :, : );

fmeTestRefinementGain(2,:,:) = fmeTest( dctInd, :, : ) - fmeTest( dcInd, :, : );
fllTestRefinementGain(2,:,:) = fllTest( dctInd, :, : ) - fllTest( dcInd, :, : );
zllTestRefinementGain(2,:,:) = zllTest( dctInd, :, : ) - zllTest( dcInd, :, : );
nllTestRefinementGain(2,:,:) = nllTest( dctInd, :, : ) - nllTest( dcInd, :, : );

fmeTrainRefinementGain(2,:,:) = fmeTrain( dctInd, :, : ) - fmeTrain( dcInd, :, : );
fllTrainRefinementGain(2,:,:) = fllTrain( dctInd, :, : ) - fllTrain( dcInd, :, : );
zllTrainRefinementGain(2,:,:) = zllTrain( dctInd, :, : ) - zllTrain( dcInd, :, : );
nllTrainRefinementGain(2,:,:) = nllTrain( dctInd, :, : ) - nllTrain( dcInd, :, : );

fmeTestRefinementGain(3,:,:) = fmeTest( mcInd, :, : ) - fmeTest( dcInd, :, : );
fllTestRefinementGain(3,:,:) = fllTest( mcInd, :, : ) - fllTest( dcInd, :, : );
zllTestRefinementGain(3,:,:) = zllTest( mcInd, :, : ) - zllTest( dcInd, :, : );
nllTestRefinementGain(3,:,:) = nllTest( mcInd, :, : ) - nllTest( dcInd, :, : );

fmeTrainRefinementGain(3,:,:) = fmeTrain( mcInd, :, : ) - fmeTrain( dcInd, :, : );
fllTrainRefinementGain(3,:,:) = fllTrain( mcInd, :, : ) - fllTrain( dcInd, :, : );
zllTrainRefinementGain(3,:,:) = zllTrain( mcInd, :, : ) - zllTrain( dcInd, :, : );
nllTrainRefinementGain(3,:,:) = nllTrain( mcInd, :, : ) - nllTrain( dcInd, :, : );

figure;
ah = axes;
plot( ah, beacons, mean( fmeTestRefinementGain(1:2,:,:), 3 )' );
set( ah, 'XTick', [2,3,4,5] );
xlabel( 'Number of beacons, N_{B}' );
ylabel( 'Test FME Improvement' );
legend( 'Dense', 'Diagonal' );

figure;
ah = axes;
plot( ah, beacons, mean( fllTestRefinementGain(1:2,:,:), 3 )' );
set( ah, 'XTick', [2,3,4,5] );
xlabel( 'Number of beacons, N_{B}' );
ylabel( 'Test FLL Improvement' );
legend( 'Dense', 'Diagonal' );

figure;
ah = axes;
plot( ah, beacons, mean( zllTestRefinementGain(1:2,:,:), 3 )' );
set( ah, 'XTick', [2,3,4,5] );
xlabel( 'Number of beacons, N_{B}' );
ylabel( 'Test ZLL Improvement' );
legend( 'Dense', 'Diagonal' );

figure;
ah = axes;
plot( ah, beacons, mean( nllTestRefinementGain(1:2,:,:), 3 )' );
set( ah, 'XTick', [2,3,4,5] );
xlabel( 'Number of beacons, N_{B}' );
ylabel( 'Test NLL Improvement' );
legend( 'Dense', 'Diagonal' );

figure;
% ah = subplot(2,1,1);
% ah = axes;
% hold( ah, 'on' );
% plot( ah, beacons, mean( nllTrainRefinementGain(1,:,:), 3 )', 'b.-', 'linewidth', 1 );
% plot( ah, beacons, mean( nllTrainRefinementGain(2,:,:), 3 )', 'r.-', 'linewidth', 1 );
% % plot( ah, beacons, mean( nllTrainRefinementGain(3,:,:), 3 )', 'g.-', 'linewidth', 1.5 );
% plot( ah, beacons, zeros(1,4), 'r-' );
% set( ah, 'XTick', [2,3,4,5] );
% xlabel( 'Number of beacons, N_{B}' );
% ylabel( 'Train NLL Improvement' );
% set( ah, 'YLim', [-0.4,0.4] );

ah = axes; %subplot(2,1,2);
hold( ah, 'on' );
plot( ah, beacons, mean( nllTestRefinementGain(1,:,:), 3 )','b.-', 'linewidth', 1 );
plot( ah, beacons, mean( nllTestRefinementGain(2,:,:), 3 )','g.-', 'linewidth', 1 );
% plot( ah, beacons, mean( nllTestRefinementGain(3,:,:), 3 )','g.-', 'linewidth', 1.5 );
plot( ah, beacons, zeros(1,4), 'r-' );
set( ah, 'XTick', [2,3,4,5] );
xlabel( 'Number of beacons, N_{B}' );
ylabel( 'Test NLL Improvement' );
% set( ah, 'YLim', [-0.4,0.4] );
 legendflex( {'Dense-True', 'Diagonal-True'}, 'ncol', 2, 'nrow', 1, 'anchor', {'n', 's'}, 'buffer', [0,10] )

%% Test-Train Performance Skew
inds = [mcInd, dcInd, mctInd, dctInd];
fmeSkew = fmeTest( inds, :, : ) - fmeTrain( inds, :, :);
zllSkew = zllTest( inds, :, : ) - zllTrain( inds, :, :);
nllSkew = nllTest( inds, :, : ) - nllTrain( inds, :, :);
fllSkew = fllTest( inds, :, : ) - fllTrain( inds, :, :);
 lw = 1;
 
figure;
th = subplot(2,1,1);
hold( th, 'on' );
plot( th, beacons, mean(nllSkew(1,:,:),3), plotStyles{inds(1)}, 'linewidth', lw );
plot( th, beacons, mean(nllSkew(2,:,:),3), plotStyles{inds(2)}, 'linewidth', lw );
plot( th, beacons, zeros(1,4), 'r-', 'linewidth', lw );
set( th, 'XTick', beacons );
ylabel( 'Noise LL Skew' );

% ah = subplot(4,1,2);
% hold( ah, 'on' );
% plot( ah, beacons, mean(fllSkew(1,:,:),3), plotStyles{inds(1)}, 'linewidth', lw );
% plot( ah, beacons, mean(fllSkew(2,:,:),3), plotStyles{inds(2)}, 'linewidth', lw );
% set( ah, 'XTick', beacons );
% ylabel( 'Filter LL' );

ah = subplot(2,1,2);
hold( ah, 'on' );
plot( ah, beacons, mean(zllSkew(1,:,:),3), plotStyles{inds(1)}, 'linewidth', lw );
plot( ah, beacons, mean(zllSkew(2,:,:),3), plotStyles{inds(2)}, 'linewidth', lw );
plot( ah, beacons, zeros(1,4), 'r-', 'linewidth', lw );
set( ah, 'XTick', beacons );
ylabel( 'Innovation LL Skew' );

% ah = subplot(4,1,4);
% hold( ah, 'on' );
% plot( ah, beacons, mean(fmeSkew(1,:,:),3), plotStyles{inds(1)}, 'linewidth', lw );
% plot( ah, beacons, mean(fmeSkew(2,:,:),3), plotStyles{inds(2)}, 'linewidth', lw );
% set( ah, 'XTick', beacons );
% ylabel( 'Filter Mean Error' );
 legendflex( th, {'MC', 'DC'}, 'ncol', 5, 'nrow', 1, 'anchor', {'n', 's'}, 'buffer', [0,10] )


%% Plot of training trace for MCIR

j = 3;
i = 2; % 3 beacons
for n = 1:numFolds
    figure;
    th = subplot( 2, 1, 1 );
    hold( th, 'on' );
    plot( th, [zllTrainingTraces{3,i,n}, zllTrainingTraces{1,i,n}], 'b.-', 'linewidth', 1 );
%     ylabel( 'ZLL' );
ylabel('Log-Likelihood');
%     subplot( 2, 1, 2 );
    plot( th, [nllTrainingTraces{3,i,n}, nllTrainingTraces{1,i,n}], 'R.-', 'linewidth', 1 );
%     ylabel( 'NLL' );
%     subplot( 4, 1, 3 );
    plot( th, [fllTrainingTraces{3,i,n}, fllTrainingTraces{1,i,n}], 'G.-', 'linewidth', 1 );
%     ylabel( 'FLL' );
    bh = subplot( 2, 1, 2 );
    plot( bh, [fmeTrainingTraces{3,i,n}, fmeTrainingTraces{1,i,n}], 'b.-', 'linewidth', 1 );
    ylabel( 'FME' );
    xlabel( 'Training iteration' );
    
     legendflex( th, {'ZLL', 'NLL', 'FLL'}, 'ncol', 5, 'nrow', 1, 'anchor', {'n', 's'}, 'buffer', [0,8] )
%     legend( 'ZLL', 'NLL', 'FLL', 'location', 'best' );
end

%% Overall Train performance vs beacons
figure;
for j = [1,3,5,6]
    th = subplot( 3,1,1 );
    hold on;
    plot( th, beacons, fmeTrainFolded(j,:), plotStyles{j}, 'linewidth', 1 );
    ylabel( 'Filter Mean Error' );    
    set( th, 'XTick', [2,3,4,5] );
    
    ah = subplot( 3,1,2 );
    hold on;
    plot( beacons, nllTrainFolded(j,:), plotStyles{j}, 'linewidth', 1 );
    ylabel( 'Noise LL' );
    set( ah, 'XTick', [2,3,4,5] );

    ah = subplot( 3,1,3 );
    hold on;
    plot( beacons, fllTrainFolded(j,:), plotStyles{j}, 'linewidth', 1 );
    ylabel( 'Filter LL' );
    xlabel( 'Number of beacons, N_{B}' );
    set( ah, 'XTick', [2,3,4,5] );

end

legendflex( th, {'MC', 'DC', 'CEM', 'AKF'}, 'ncol', 4, 'nrow', 1, 'anchor', {'n', 's'}, 'buffer', [0,8] )

% legend( methodNames{:}, 'location', 'best' );
%% Test performance
figure;
for j = [1,3,5,6]
    th = subplot( 3,1,1 );
    hold on;
    plot( th, beacons, fmeTestFolded(j,:), plotStyles{j}, 'linewidth', 1 );
    ylabel( 'FME' );   
    set( th, 'XTick', [2,3,4,5] );
    
    ah = subplot( 3,1,2 );
    hold on;
    plot( beacons, nllTestFolded(j,:), plotStyles{j}, 'linewidth', 1 );
    ylabel( 'NLL' );
    set( ah, 'XTick', [2,3,4,5] );

    ah = subplot( 3,1,3 );
    hold on;
    plot( beacons, fllTestFolded(j,:), plotStyles{j}, 'linewidth', 1 );
    ylabel( 'FLL' );
    xlabel( 'Number of beacons, N_{B}' );
    set( ah, 'XTick', [2,3,4,5] );
end

legendflex( th, {'MC', 'DC', 'CEM', 'AKF'}, 'ncol', 4, 'nrow', 1, 'anchor', {'n', 's'}, 'buffer', [0,8] )

%% Generate representative plot of error samples

nBeacons = 2;
n = 4;

load( [trialRoot, 'BeaconCartSim_', num2str(nBeacons), 'Beacons_Results_Fold2.mat'] );

trainSet = { [trialRoot, 'BeaconCartSim_', num2str(nBeacons), 'Beacons_Trial', num2str(n), '.mat'] };
mcirRun = RunBeaconResults( trainSet, mcirResults(1).regressor );
dcirRun = RunBeaconResults( trainSet, dcirResults(1).regressor );
cemrRun = RunBeaconResults( trainSet, cemrResults(1).regressor );
est = WindowedCovarianceEstimator( 2*nBeacons, baselineResults.bestWindow );
baselineRun = RunBeaconBaselines( trainSet, est );

T = size( mcirRun.filter.prioriR, 3 );
lw = 1.25;

figure;
th = subplot(2,1,1);
hold( th, 'on' );
i = 1;
plot( th, mcirRun.true.errors(i,:), 'k.' );

plot( th, 3*sqrt(squeeze(mcirRun.filter.prioriR(i,i,:))), 'b-', 'linewidth', lw );
plot( th, 3*sqrt(squeeze(dcirRun.filter.prioriR(i,i,:))), 'g-', 'linewidth', lw );
plot( th, 3*sqrt(squeeze(cemrRun.filter.prioriR(i,i,:))), 'k-', 'linewidth', lw );
plot( th, 3*sqrt(squeeze(baselineRun.adaptive.adaptedR(i,i,:))), 'r-', 'linewidth', lw );

plot( th, -3*sqrt(squeeze(mcirRun.filter.prioriR(i,i,:))), 'b-', 'linewidth', lw );
plot( th, -3*sqrt(squeeze(dcirRun.filter.prioriR(i,i,:))), 'g-', 'linewidth', lw );
plot( th, -3*sqrt(squeeze(cemrRun.filter.prioriR(i,i,:))), 'k-', 'linewidth', lw );
plot( th, -3*sqrt(squeeze(baselineRun.adaptive.adaptedR(i,i,:))), 'r-', 'linewidth', lw );

% legend( '\eta', 'MC', 'CEM', 'AKF', 'location', 'best' );
% xlabel( 'Timestep, t' );
ylabel( ['Range error, r_{', num2str((i-1)/2 + 1), '}'] );
axis( th, [0, T -4 4] );

ah = subplot(2,1,2);
hold( ah, 'on' );
% i = 1;
plot( ah, mcirRun.true.errors(i+1,:), 'k.' );

plot( ah, 3*sqrt(squeeze(mcirRun.filter.prioriR(i+1,i+1,:))), 'b-', 'linewidth', lw );
plot( ah, 3*sqrt(squeeze(dcirRun.filter.prioriR(i+1,i+1,:))), 'g-', 'linewidth', lw );
plot( ah, 3*sqrt(squeeze(cemrRun.filter.prioriR(i+1,i+1,:))), 'k-', 'linewidth', lw );
plot( ah, 3*sqrt(squeeze(baselineRun.adaptive.adaptedR(i+1,i+1,:))), 'r-', 'linewidth', lw );

plot( ah, -3*sqrt(squeeze(mcirRun.filter.prioriR(i+1,i+1,:))), 'b-', 'linewidth', lw );
plot( ah, -3*sqrt(squeeze(dcirRun.filter.prioriR(i+1,i+1,:))), 'g-', 'linewidth', lw );
plot( ah, -3*sqrt(squeeze(cemrRun.filter.prioriR(i+1,i+1,:))), 'k-', 'linewidth', lw );
plot( ah, -3*sqrt(squeeze(baselineRun.adaptive.adaptedR(i+1,i+1,:))), 'r-', 'linewidth', lw );

legendflex( th, {'\epsilon', 'MC', 'DC', 'CEM', 'AKF'}, 'ncol', 5, 'nrow', 1, 'anchor', {'n', 's'}, 'buffer', [0,8] )
xlabel( 'Timestep, t' );
ylabel( ['Bearing error, \psi_{', num2str((i+1)/2), '} (rad)'] );
axis( ah, [0, T -0.5 0.5] );

% for n = 1:1
%     trainSet = { [trialRoot, 'BeaconCartSim_', num2str(nBeacons), 'Beacons_Trial', num2str(n), '.mat'] };
%     mcirRun = RunBeaconResults( trainSet, mcirResults(1).regressor );
%     %     dcirRun = RunBeaconResults( trainSet, dcirResults(1).regressor );
%     cemrRun = RunBeaconResults( trainSet, cemrResults(1).regressor );
%     est = WindowedCovarianceEstimator( 2*nBeacons, baselineResults.bestWindow );
%     baselineRun = RunBeaconBaselines( trainSet, est );
%     
%     for i = 1:2:2*nBeacons
%         figure;
%         ah = axes;
%         hold( ah, 'on' );
%         
%         plot( ah, mcirRun.true.errors(i,:), 'k.' );
%         
%         plot( ah, 3*sqrt(squeeze(mcirRun.filter.prioriR(i,i,:))), 'b-' );
%         plot( ah, 3*sqrt(squeeze(cemrRun.filter.prioriR(i,i,:))), 'g-' );
%         plot( ah, 3*sqrt(squeeze(baselineRun.adaptive.adaptedR(i,i,:))), 'r-' );
%         
%         plot( ah, -3*sqrt(squeeze(mcirRun.filter.prioriR(i,i,:))), 'b-' );
%         plot( ah, -3*sqrt(squeeze(cemrRun.filter.prioriR(i,i,:))), 'g-' );
%         plot( ah, -3*sqrt(squeeze(baselineRun.adaptive.adaptedR(i,i,:))), 'r-' );
%         
%         legend( '\eta', 'MC', 'CEM', 'AKF', 'location', 'best' );
%         xlabel( 'Timestep, t' );
%         ylabel( ['Range, r_{', num2str((i-1)/2 + 1), '}'] );
%         
%         figure;
%         ah = axes;
%         hold( ah, 'on' );
%         
%         plot( ah, mcirRun.true.errors(i+1,:), 'k.' );
%         
%         plot( ah, 3*sqrt(squeeze(mcirRun.filter.prioriR(i+1,i+1,:))), 'b-' );
%         plot( ah, 3*sqrt(squeeze(cemrRun.filter.prioriR(i+1,i+1,:))), 'g-' );
%         plot( ah, 3*sqrt(squeeze(baselineRun.adaptive.adaptedR(i+1,i+1,:))), 'r-' );
%         
%         plot( ah, -3*sqrt(squeeze(mcirRun.filter.prioriR(i+1,i+1,:))), 'b-' );
%         plot( ah, -3*sqrt(squeeze(cemrRun.filter.prioriR(i+1,i+1,:))), 'g-' );
%         plot( ah, -3*sqrt(squeeze(baselineRun.adaptive.adaptedR(i+1,i+1,:))), 'r-' );
%         
%         legend( '\eta', 'MC', 'CEM', 'AKF', 'location', 'best' );
%         xlabel( 'Timestep, t' );
%         ylabel( ['Bearing, \psi_{', num2str((i+1)/2), '}'] );
%         
%     end
% end

%% Split processing
trialRoot = '../data/beaconTrials/';
% trialRoot = 'C:/Users/Humhu/Dropbox/Research/Papers/rss2015/data/beaconTrials/';

load( [trialRoot, 'BeaconDatasetParams.mat' ] );

testParams = testParams(2); % B = 3
beacons = [testParams.nBeacons];

numFolds = 4;

fmeTrain = zeros( numMethods, numParamSets, numFolds );
fllTrain = zeros( numMethods, numParamSets, numFolds );
zllTrain = zeros( numMethods, numParamSets, numFolds );
nllTrain = zeros( numMethods, numParamSets, numFolds );

fmeTest = zeros( numMethods, numParamSets, numFolds );
fllTest = zeros( numMethods, numParamSets, numFolds );
zllTest = zeros( numMethods, numParamSets, numFolds );
nllTest = zeros( numMethods, numParamSets, numFolds );

nBeacons = testParams.nBeacons;

resultsName = [trialRoot, 'BeaconCartSim_', num2str(nBeacons), ...
    'Beacons_Results_Splitted_Folded.mat'];
fprintf( ['Parsing results from ', resultsName, '\n'] );

res = load( resultsName );

numSplits = testParams.nIters*0.75;
for k = 1:3:numSplits
    for n = 1:numFolds
        for j = 1:numMethods-1
            
            methodTest = ['res.', methods{j}, 'Results(', num2str(k), ',', num2str(n), ').test_eval'];
            methodTrain = ['res.', methods{j}, 'Results(', num2str(k), ',', num2str(n), ').train_eval'];
            
            fmeTest(j,k,n) = eval( [methodTest, '.filter.x_avg_err'] );
            fllTest(j,k,n) = eval( [methodTest, '.filter.x_avg_ll'] );
            zllTest(j,k,n) = eval( [methodTest, '.filter.z_avg_ll'] );
            nllTest(j,k,n) = eval( [methodTest, '.true.n_avg_ll'] );
            
            zTemp = eval( ['[res.', methods{j}, 'Results(', num2str(k), ',', num2str(n), ').train.evals.filter]'] );
            nTemp = eval( ['[res.', methods{j}, 'Results(', num2str(k), ',', num2str(n), ').train.evals.true]'] );
            zllTrainingTraces{j,k,n} = [zTemp.z_avg_ll];
            nllTrainingTraces{j,k,n} = [nTemp.n_avg_ll];
            fllTrainingTraces{j,k,n} = [zTemp.x_avg_ll];
            
            fmeTrain(j,k,n) = eval( [methodTrain, '.filter.x_avg_err'] );
            fllTrain(j,k,n) = eval( [methodTrain, '.filter.x_avg_ll'] );
            zllTrain(j,k,n) = eval( [methodTrain, '.filter.z_avg_ll'] );
            nllTrain(j,k,n) = eval( [methodTrain, '.true.n_avg_ll'] );
            
            
        end
        fmeTest(end,k,n) = res.baselineResults(k,n).test_eval.adaptive.x_avg_err;
        fllTest(end,k,n) = res.baselineResults(k,n).test_eval.adaptive.x_avg_ll;
        zllTest(end,k,n) = res.baselineResults(k,n).test_eval.adaptive.z_avg_ll;
        nllTest(end,k,n) = res.baselineResults(k,n).test_eval.adaptive.n_avg_ll;
        
        fmeTrain(end,k,n) = res.baselineResults(k,n).train_eval.adaptive.x_avg_err;
        fllTrain(end,k,n) = res.baselineResults(k,n).train_eval.adaptive.x_avg_ll;
        zllTrain(end,k,n) = res.baselineResults(k,n).train_eval.adaptive.z_avg_ll;
        nllTrain(end,k,n) = res.baselineResults(k,n).train_eval.adaptive.n_avg_ll;
    end
end

fmeTestFolded = mean( fmeTest, 3 );
fllTestFolded = mean( fllTest, 3 );
zllTestFolded = mean( zllTest, 3 );
nllTestFolded = mean( nllTest, 3 );

fmeTrainFolded = mean( fmeTrain, 3 );
fllTrainFolded = mean( fllTrain, 3 );
zllTrainFolded = mean( zllTrain, 3 );
nllTrainFolded = mean( nllTrain, 3 );
%% Performance vs data
figure;
splits = (1:3:numSplits)/testParams.nIters;
for j = [1,3,5,6]
    th = subplot( 3,1,1 );
    hold on;
    plot( th, splits, fmeTestFolded(j,1:3:end), plotStyles{j}, 'linewidth', 1.5 );
    ylabel( 'Filter Mean Error' );    
%     set( ah, 'XTick', [2,3,4,5], 'YLim', [0, 0.31] );
    
    ah = subplot( 3,1,2 );
    hold on;
    plot( splits, nllTestFolded(j,1:3:end), plotStyles{j}, 'linewidth', 1.5 );
    ylabel( 'Noise LL' );
%     set( ah, 'XTick', [2,3,4,5] );

    ah = subplot( 3,1,3 );
    hold on;
    plot( splits, fllTestFolded(j,1:3:end), plotStyles{j}, 'linewidth', 1.5 );
    ylabel( 'Filter LL' );
    xlabel( 'Fraction of data used to train' );
%     set( ah, 'XTick', [2,3,4,5] );
end

legendflex( th, {'MC', 'DC', 'CEM', 'AKF'}, 'ncol', 4, 'nrow', 1, 'anchor', {'n', 's'}, 'buffer', [0,8] )
% legend( methodNames{:}, 'location', 'best' );
