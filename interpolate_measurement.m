function [arm_estimate] = interpolate_measurement(knn_search_object, ...
    april_poses, associated_arm_poses, ...
    num_nearest_neighbors, april_measurement)
% Returns the estimated arm position from a given april_measurement

    % Lookup a thing, and do interpolation
    [idx, dist] = knnsearch(knn_search_object, april_measurement', 'k', ...
        num_nearest_neighbors);

    k_arm_poses = associated_arm_poses(:, idx);
    k_april_poses = april_poses(:, idx);
    
    % Use weighted average
    % Make zeros just barely non-zero
    dist_equal_zero = (dist == 0);
    dist(dist_equal_zero) = 0.000001;
    
    % Simple Averaging
    %k_arm_pose_weights = 1 ./ dist;
    %k_arm_pose_weights = k_arm_pose_weights / sum(k_arm_pose_weights);
    %k_weighted_arm_poses = zeros(7,1);
    %for i = 1:size(k_arm_pose_weights, 2)
    %   k_weighted_arm_poses = k_weighted_arm_poses + ...
    %            k_arm_poses(:, i) * k_arm_pose_weights(i); 
    %end
    
    % Intelligent averaging using Multivariate normal probability
    %  density. 
    % mvnpdf: Multivariate normal probability density function
    %  y = mvnpdf(X, {mu, {sigma}})
    %   X = nx1 vector, containing the PDF of the multivariate
    %       normal distribution with zero mean and identity 
    %       covariance matrix. 
    %  (TODO the covariance can be tuned to get better results.)
    mean = april_measurement(4:9, :)'
    cov = eye(6) / 5000;
    measurements = k_april_poses(4:9, :)'
    k_arm_pose_weights = mvnpdf(measurements, mean, cov)';
    k_arm_pose_weights = k_arm_pose_weights / sum(k_arm_pose_weights);
    k_weighted_arm_poses = zeros(7,1);
    for i = 1:size(k_arm_pose_weights, 2)
       k_weighted_arm_poses = k_weighted_arm_poses + ...
                k_arm_poses(:, i) * k_arm_pose_weights(i); 
    end
    
    % Put together weighted averages for the full estimate
    arm_estimate = k_weighted_arm_poses;



end

