

% Perform lookups
Y = aprilposes(:, 2)';
[idx, dist] = knnsearch(knn_search_object, Y, 'k', 5);

k_arm_poses = armposes(:, idx);
k_arm_pose_dist = dist;



% Perform linear integration....
