function [eval] = EvaluateResults( results )
% Calculates various metrics from a results struct matrix
% Returns an eval(i) struct with the following fields:
%   filter - A struct with filter results:
%       x_traj_err - Error norms along trajectory
%       x_avg_err - Mean error norm from ground truth
%       x_max_err - Max error norm along trajectory
%       x_traj_ll - Trajectory true pose trajectory log likelihood
%       x_avg_ll - Average trajectory true pose log likelihood
%       x_min_ll - Min true pose log likelihood along trajectory
%       z_traj_ll - Trajectory innovation log likelihood
%       z_avg_ll - Average innovation log likelihood
%       z_min_ll - Min innovation log likelihood along trajectory
%       n_traj_ll - Trajectory true noise log likelihood
%       n_avg_ll - Average true noise log likelihood
%       n_min_ll - Min true noise log likelihood
%   smoother - A struct with smoother results:
%       x_traj_err - Error norms along trajectory
%       x_avg_err - Mean error norm from ground truth
%       x_max_err - Max error norm along trajectory
%       x_traj_ll - True pose trajectory log likelihood
%       x_avg_ll - Average true pose log likelihood
%       x_min_ll - Min true pose log likelihood along trajectory
%       r_ll - Residual log likelihood
%       r_avg_ll - Average residual log likelihood
%       r_min_ll - Min residual log likelihood along trajectory
%       n_traj_ll - Trajectory true noise log likelihood
%       n_avg_ll - Average true noise log likelihood
%       n_min_ll - Min true noise log likelihood
%   true - A struct with ground true results:
%       n_traj_ll - Trajectory true noise log likelihood
%       n_avg_ll - Average true noise log likelihood
%       n_min_ll - Min true noise log likelihood

numRegressors = numel( results );

for i = 1:numRegressors
    % 1. Calculate filter results
    filterErrs = results(i).difference( results(i).filter.means, results(i).true.poses );
    eval(i).filter.x_traj_err = sqrt( sum( filterErrs.*filterErrs, 1 ) );
    eval(i).filter.x_avg_err = mean( eval(i).filter.x_traj_err );
    eval(i).filter.x_max_err = max( eval(i).filter.x_traj_err );
    
    eval(i).filter.x_traj_ll = ...
        GaussianLogLikelihood( filterErrs, results(i).filter.covariances, 'covariance' );
    eval(i).filter.x_avg_ll = mean( eval(i).filter.x_traj_ll );
    eval(i).filter.x_min_ll = min( eval(i).filter.x_traj_ll );
    
    eval(i).filter.z_traj_ll = ...
        GaussianLogLikelihood( results(i).filter.innovations, ...
        results(i).filter.prioriCovariances + results(i).filter.prioriR, 'covariance' );
    eval(i).filter.z_avg_ll = mean( eval(i).filter.z_traj_ll );
    eval(i).filter.z_min_ll = min( eval(i).filter.z_traj_ll );
    
    eval(i).filter.n_traj_ll = ...
        GaussianLogLikelihood( results(i).true.errors, results(i).filter.posterioriR, 'covariance' );
    eval(i).filter.n_avg_ll = mean( eval(i).filter.n_traj_ll );
    eval(i).filter.n_min_ll = min( eval(i).filter.n_traj_ll );
    
    % 2. Calculate smoother results(i)
    smootherErrs = results(i).difference( results(i).smoother.means, results(i).true.poses );
    eval(i).smoother.x_traj_err = sqrt( sum( smootherErrs.*smootherErrs, 1 ) );
    eval(i).smoother.x_avg_err = mean( eval(i).smoother.x_traj_err );
    eval(i).smoother.x_max_err = max( eval(i).smoother.x_traj_err );
    
    eval(i).smoother.x_traj_ll = ...
        GaussianLogLikelihood( smootherErrs, results(i).smoother.covariances, 'covariance' );
    eval(i).smoother.x_avg_ll = mean( eval(i).smoother.x_traj_ll );
    eval(i).smoother.x_min_ll = min( eval(i).smoother.x_traj_ll );
    
    eval(i).smoother.r_traj_ll = ...
        GaussianLogLikelihood( results(i).smoother.residuals, ...
        results(i).smoother.mappedCovariances + results(i).smoother.R, 'covariance' );
    eval(i).smoother.r_avg_ll = mean( eval(i).smoother.r_traj_ll );
    eval(i).smoother.r_min_ll = min( eval(i).smoother.r_traj_ll );
    
    eval(i).smoother.n_traj_ll = ...
        GaussianLogLikelihood( results(i).true.errors, results(i).smoother.R, 'covariance' );
    eval(i).smoother.n_avg_ll = mean( eval(i).smoother.n_traj_ll );
    eval(i).smoother.n_min_ll = min( eval(i).smoother.n_traj_ll );
    
    % 3. Calculate true results(i)
    eval(i).true.n_traj_ll = ...
        GaussianLogLikelihood( results(i).true.errors, results(i).true.R, 'covariance' );
    eval(i).true.n_avg_ll = mean( eval(i).true.n_traj_ll );
    eval(i).true.n_min_ll = min( eval(i).true.n_traj_ll );
end

