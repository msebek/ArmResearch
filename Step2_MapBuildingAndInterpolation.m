
filename = 'arm_april_data_first_set';

load(filename, 'aprilposes', 'armposes');

% April Pose: [x; y; z; roll; pitch; yaw]
% Arm Pose: [x; y; z; q_0; q_1; q_2; q_3]

% All observed april poses; rows are observations
% Hashmap of april poses to arm poses
% Converts each column to a cell
%arm_row_sizes = ones(1, size(armposes, 2));
%arm_col_sizes = [7];
%april_row_sizes = ones(1, size(aprilposes, 2));
%april_col_sizes = [9];

% These are the values (in cells)
%arm_cells = mat2cell(armposes', arm_row_sizes, arm_col_sizes);

% These are the keys (as strings)
%april_arm_map = containers.Map(april_poses_string, arm_cells);
knn_search_object = createns(aprilposes', 'Distance', 'euclidean', ...
        'NSMethod', 'kdtree');


% Perform lookups
Y = aprilposes(:, 2)';
[idx, dist] = knnsearch(knn_search_object, Y, 'k', 5);

k_arm_poses = armposes(:, idx);
k_arm_pose_dist = dist;



% Perform linear integration....
