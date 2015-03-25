function [bestWindow, res] = TrainBaseline( evalFunc, trainingParams )
% Selects the adaptive KF window size using the evaluation function
% evalFunc - A function handle that takes a  and returns a
%   results struct
% regressor - The initial regressor
% trainingParams - A struct with fields
%   type - 'innovation', 'error', or 'EM' for fitting approach
%   delTolerance - smallest average norm difference between parameters for
%                  convergence
%
% Details of methods:
%   'innovation' training mode first fits the regressor to maximize the
%       innovation sequence likelihood, then reruns the filter. The process
%       is iterated until convergence.
%   'error' training mode fits the regressor to the true errors directly.
%   'EM' training mode estimates the error using the smoother states, then
%       fits a model and reruns the smoother, iterating until convergence.

numWindows = numel( trainingParams.windows );
bestWindow = 0;
bestRLL = -Inf;
res = [];

for w = 1:numWindows
    
    window = trainingParams.windows(w);    
    
    % TODO Ugly!
    clear estimator;
    estimator( trainingParams.numEstimators ) = WindowedCovarianceEstimator;
    for i = 1:trainingParams.numEstimators
        estimator(i) = WindowedCovarianceEstimator( trainingParams.zDim, window );
        estimator(i).Initialize( trainingParams.initCov );
    end
    
    result = evalFunc( estimator );
    eval = EvaluateBaselineResults( result );
    
    fprintf( 'AKF [%d] NLL: %f ZLL: %f RLL: %f FME: %f FLL %f\n', ...
        window, eval.adaptive.n_avg_ll, eval.adaptive.z_avg_ll, ...
            eval.adaptive.r_avg_ll, eval.adaptive.x_avg_err, ...
            eval.adaptive.x_avg_ll );
    
    if eval.adaptive.r_avg_ll > bestRLL
        bestRLL = eval.adaptive.r_avg_ll;
        bestWindow = window;
    end
    
    if isempty(res)
        res = eval;
    else
        res(end+1) = eval;
    end
    
end