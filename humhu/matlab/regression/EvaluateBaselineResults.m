function [eval] = EvaluateBaselineResults( results )
% Calculates various metrics from a results struct matrix
% Returns an eval struct with the following fields:
%   adaptive - A struct with adaptive filter results:
%       x_traj_err - Error norms along trajectory
%       x_avg_err - Mean error norm from ground truth
%       x_max_err - Max error norm along trajectory
%       x_traj_ll - Average true pose trajectory log likelihood
%       x_min_ll - Min true pose log likelihood along trajectory
%       r_traj_ll - Trajectory residual log likelihood
%       r_avg_ll - Average residual log likelihood
%       r_min_ll - Min residual log likelihood
%       z_traj_ll - Trajectory innovation log likelihood
%       z_avg_ll - Average innovation log likelihood
%       z_min_ll - Min innovation log likelihood along trajectory
%       n_traj_ll - Trajectory true noise log likelihood
%       n_avg_ll - Average true noise log likelihood
%       n_min_ll - Min true noise log likelihood

% 1. Calculate filter results
adaptiveErrs = results.difference( results.adaptive.means, results.true.poses );
eval.adaptive.x_traj_err = sqrt( sum( adaptiveErrs.*adaptiveErrs, 1 ) );
eval.adaptive.x_avg_err = mean( eval.adaptive.x_traj_err );
eval.adaptive.x_max_err = max( eval.adaptive.x_traj_err );

eval.adaptive.x_traj_ll = ...
    GaussianLogLikelihood( adaptiveErrs, results.adaptive.covariances, 'covariance' );
eval.adaptive.x_avg_ll = mean( eval.adaptive.x_traj_ll );
eval.adaptive.x_min_ll = min( eval.adaptive.x_traj_ll );

eval.adaptive.r_traj_ll = ...
    GaussianLogLikelihood( results.adaptive.residuals, results.adaptive.adaptedR, 'covariance' );
eval.adaptive.r_avg_ll = mean( eval.adaptive.r_traj_ll );
eval.adaptive.r_min_ll = min( eval.adaptive.r_traj_ll );

eval.adaptive.z_traj_ll = ...
    GaussianLogLikelihood( results.adaptive.innovations, ...
    results.adaptive.prioriCovariances + results.adaptive.adaptedR, 'covariance' );
eval.adaptive.z_avg_ll = mean( eval.adaptive.z_traj_ll );
eval.adaptive.z_min_ll = min( eval.adaptive.z_traj_ll );

eval.adaptive.n_traj_ll = ...
    GaussianLogLikelihood( results.true.errors, results.adaptive.adaptedR, 'covariance' );
eval.adaptive.n_avg_ll = mean( eval.adaptive.n_traj_ll );
eval.adaptive.n_min_ll = min( eval.adaptive.n_traj_ll );
    

