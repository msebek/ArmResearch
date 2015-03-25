function [tag, regressorResults] = RunArmTrial( trajectory, regressor, trainingParams, EMparams )
% Runs an EM loop to estimate the tag position and fit an observation noise
% regressor. Returns the following structs:
%   tag - Struct with iterated tag results with fields:
%       mean - Tag estimate mean
%       covariance - Tag estimate covariance
%   regressorResults - Struct with the iterated regressor results:
%       regressor - Trained regressor
%       train - Training evaluation trace
% regressorResults(i) are generated using tag(i). There should be one more
% tag result than regressor.

% First estimate tag prior
tagPoses = [trajectory.cameraPose]*[trajectory.tagMeasurement];
tagPoseMean = SE3( mean( tagPoses.GetCoordinates(), 2 ) );
tag.mean = tagPoseMean;
tagPoseErrs = tagPoseMean\tagPoses;
tag.covariance = cov( tagPoseErrs.GetCoordinates()' );

iters = 0;
tags = tag;
regressorResults = [];

while iters < EMparams.maxIters
    iters = iters + 1;
    
    trainEvalFunc = @(regressor) RunArmResults( trajectory, regressor, tag );
    
    regRes = [];
    [regRes.regressor, regRes.train] = TrainRegressor( trainEvalFunc, regressor, trainingParams );
    if isempty( regressorResults )
        regressorResults = regRes;
    else
        regressorResults(end+1) = regRes;
    end
    
    tagFilter = KalmanFilterSE3();
    tagFilter.Initialize( tagPoseMean, 1E3*eye(3) );
    tagFilter.SetTransitionCovariance( 1E-6*eye(6) );
    tagNew = RunTagFilter( tagFilter, trajectory, regressor ); 
    tagDel = tag.mean/tagNew;
    tagDel = norm( tagDel.GetCoordinates() );
    
    tag = tagNew;
    tags(end+1) = tag;
    if tagDel < EMparams.minDel 
        break
    end
    
end