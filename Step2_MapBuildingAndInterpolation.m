
filename = 'arm_april_data_first_set';

load(filename, 'aprilposes', 'armposes');

% April Pose: [x; y; z; roll; pitch; yaw]
% Arm Pose: [x; y; z; q_0; q_1; q_2; q_3]

% 1. Process both of the pose sets into homogeneous rotation matrices.
% Get a homogeneous matrix for each april pose
%  Hac = Transform from the april tag to the camera, in camera
%           coordinates. 
Hac = zeros(4, 4);
for i=1:size(aprilposes, 2)
    Hac(:, :, i) = my_rpytohomog(aprilposes(:, i));
end

% Get a homogeneous matrix for each arm pose
Hew = zeros(4, 4);
for i=1:size(armposes, 2)
    % Note that quat2mat expects [x; y; z; w], and 
    %  armposes have [x, y, z, qw, qx, qy, qz] 
    Hew(1:3, 1:3, i) = quat2mat([armposes(5:7, i); armposes(4, i)]);
    
    % Note that the arm poses are measured in MM
    % Divide by 1000 to get meters.
    Hew(1:3, 4, i) = armposes(1:3, i) / 1000;
    Hew(4, 4, i) = 1;
end

% 2. Determine H from tag to the end effector by using the ground
%     truth to the tag. 
% TODO ask trevor for the real sheet
% The ground truth; the arm pose s.t. the april pose is 0. 
arm_ground_x = 89.61 / 1000; % meters
arm_ground_y = ~1412.94 / 1000;
arm_ground_z = 810.48 / 1000;
arm_ground_q0_w = 0.409; % radians
arm_ground_q1_x = 0.49;
arm_ground_q2_y = 0.621;
arm_ground_q3_z = -.454;

% Find the tag ground truth (arm pose at tag)
Hgw = my_quattohomog([arm_ground_x, arm_ground_y, arm_ground_z, ...
                    arm_ground_q0_w, arm_ground_q1_x, arm_ground_q2_y, ...
                    arm_ground_q3_z]');

% Find Hge (transform from ground truth of arm to end
%           effector in end-effector frame.)
%           (this should be similar to the tag's april position)
Hge = zeros(4, 4);
for i=1:size(armposes, 2)
    % H_g^e = pinv(H^w_e)*H^W_g
    Hge(:, :, i) = pinv(Hew(:,:,i)) * Hgw;
end
  
% TODO verify that some of them look the same....
%Hge(:,:,1)
%Hac(:,:,1)


% 3. Create a KDtree of the april poses to actual position. 
% All observed april poses; rows are observations
% These are the keys (as strings)
knn_search_object = createns(aprilposes', 'Distance', 'euclidean', ...
        'NSMethod', 'kdtree');


