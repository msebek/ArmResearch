function [regressors, res] = TrainRegressor( evalFunc, regressors, trainingParams )
% Trains a regressor using the evaluation function and parameters
% evalFunc - A function handle that takes the regressors and returns a
%   results struct array for each regressor
% regressors - The initial regressors struct array
% trainingParams - A struct array with fields for each regressor:
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

options = optimoptions( 'fminunc', 'algorithm', 'quasi-newton', 'GradObj','on', 'tolfun', 0, 'Display', 'off' );

numRegressors = numel( regressors );

iters = zeros(numRegressors,1);
while true
    
    iterationResults = evalFunc( regressors );
    iters = iters + 1;
    
    iterationEvaluations = EvaluateResults( iterationResults );
    res.evals(iters) = iterationEvaluations;
    
    for i = 1:numRegressors
        
        if iters(i) > trainingParams(i).maxIters
            break;
        end
        
        regressorResult = iterationResults(i);
        regressorEvaluation = iterationEvaluations(i);
        
        if strcmp( trainingParams(i).type, 'innovation' )
            
            % TODO: Train with priori or posteriori features here?
            [regressors(i), out] = FitRegressor( regressorResult.filter.innovations, ...
                regressorResult.filter.prioriFeatures, regressorResult.filter.prioriCovariances, regressors(i), options );
            
            fprintf( [regressors(i).type, ' [innov] ID: %d Iter: %d Del: %f NLL: %f ZLL: %f FME: %f FLL: %f\n'], ...
                i, iters(i), out.del, regressorEvaluation.true.n_avg_ll, regressorEvaluation.filter.z_avg_ll, regressorEvaluation.filter.x_avg_err, regressorEvaluation.filter.x_avg_ll );
            if out.del < trainingParams(i).delTolerance
                break;
            end
            
        elseif strcmp( trainingParams(i).type, 'residual' )
            
            [regressors(i), out] = FitRegressor( regressorResult.filter.residuals, ...
                regressorResult.filter.posterioriFeatures, -regressorResult.filter.posterioriCovariances, regressors(i), options );
            
            fprintf( [regressors(i).type, ' [resi] iter %d Del: %f NLL: %f RLL: %f FME: %f FLL: %f\n'], ...
                iters(i), out.del, regressorEvaluation.true.n_avg_ll, regressorEvaluation.filter.r_avg_ll, regressorEvaluation.filter.x_avg_err, regressorEvaluation.filter.x_avg_ll );
            if out.del < trainingParams(i).delTolerance
                break;
            end
            
        elseif strcmp( trainingParams(i).type, 'error' )
            
            [regressors(i), out] = FitRegressor( regressorResult.true.errors, regressorResult.true.features, ...
                [], regressors(i), options );
            
            regressorResult = evalFunc( regressors(i) );
            regressorEvaluation = EvaluateResults( regressorResult );
            res.evals(iters(i)) = regressorEvaluation;
            
            fprintf( [regressors(i).type, ' [err] iter %d Del: %f NLL: %f \tSME: %f SLL: %f\n'], ...
                iters(i), out.del, regressorEvaluation.true.n_avg_ll, regressorEvaluation.smoother.x_avg_err, regressorEvaluation.smoother.x_avg_ll );
            break;
            
            %     elseif strcmp( trainingParams.type, 'EM' )
            %
            %         emIters = 0;
            %         k = 1;
            %         %         while emIters < trainingParams.maxEMAttempts
            %         %             try
            %         [regressor, out] = FitRegressor( iterationResults.smoother.residuals, ...
            %             iterationResults.smoother.features, iterationResults.smoother.mappedCovariances, regressor, options );
            %         %                 break;
            %         %             catch err
            %         %                 k = k*trainingParams.emAlpha;
            %         %                 emIters = emIters + 1;
            %         %             end
            %         %         end
            %
            %         fprintf( [regressor.type, ' [EM] iter %d Del: %f NLL: %f RLL %f SME: %f SLL: %f k: %d \n'], ...
            %             iter, out.del, evaluation.true.n_avg_ll, evaluation.smoother.r_avg_ll, ...
            %             evaluation.smoother.x_avg_err, evaluation.smoother.x_avg_ll, k );
            %         if out.del/k < trainingParams.delTolerance
            %             break;
            %         end
        elseif strcmp( trainingParams(i).type, 'mle' )
            
            T = size( regressorResult.smoother.residuals, 2 );
            nR = cov( regressorResult.smoother.residuals' ) + sum( regressorResult.smoother.mappedCovariances, 3 )/T;
            
            del = norm( regressors(i).R(:) - nR(:) );
            regressors(i).R = nR;
            fprintf( [regressors(i).type, ' [MLE] iter %d Del: %f NLL: %f RLL %f SME: %f SLL: %f \n'], ...
                iters(i), del, regressorEvaluation.true.n_avg_ll, regressorEvaluation.smoother.r_avg_ll, ...
                regressorEvaluation.smoother.x_avg_err, regressorEvaluation.smoother.x_avg_ll );
            
            out.fval = 0;
            if del < trainingParams(i).delTolerance
                break;
            end
            
        else
            error( 'Invalid training type.' );
        end
    end
end

res.iters = iters;
res.fval = out.fval;