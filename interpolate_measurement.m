function [arm_estimate] = interpolate_measurement(knn_search_object, ...
    associated_arm_poses, num_nearest_neighbors, april_measurement)
% Returns the estimated arm position from a given april_measurement

    % Lookup a thing, and do interpolation
    [idx, dist] = knnsearch(knn_search_object, april_measurement', 'k', ...
        num_nearest_neighbors);

    k_arm_poses = associated_arm_poses(:, idx);

    % Use weighted average
    % Make zeros just barely non-zero
    dist_equal_zero = (dist == 0);
    dist(dist_equal_zero) = 0.000001;
    
    %
    k_arm_pose_weights = 1 ./ dist;
    k_arm_pose_weights = k_arm_pose_weights / sum(k_arm_pose_weights);
    k_weighted_arm_poses = zeros(7,1);
    for i = 1:size(k_arm_pose_weights, 2)
       k_weighted_arm_poses = k_weighted_arm_poses + ...
                k_arm_poses(:, i) * k_arm_pose_weights(i); 
    end

    % Put together weighted averages for the full estimate
    arm_estimate = k_weighted_arm_poses;



end

