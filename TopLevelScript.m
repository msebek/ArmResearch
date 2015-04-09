% Load all the data files. 
filename = 'arm_april_data_second_set';
load(filename, 'aprilposes', 'armposes');

% Process them. 


% Split them into groups (later)


% 3. Create a KDtree of the april poses to actual position. 
% All observed april poses; rows are observations
% These are the keys (as strings)

first_april = aprilposes(:, 1:2:end); % odd
second_april = aprilposes(:, 2:2:end); % even
first_arm = armposes(:, 1:2:end); % odd
second_arm = armposes(:, 2:2:end); % even

% Think about different averaging functions

first_search = createns(first_april', 'Distance', 'euclidean', ...
        'NSMethod', 'kdtree');

second_search = createns(second_april', 'Distance', 'euclidean', ...
        'NSMethod', 'kdtree');

   
error = zeros(1, size(aprilposes, 2));
% Check out error between data sets.
for i=1:size(second_april, 2)
    num_neighbors = 10;
    new_measurement = second_april(:, i);
    arm_estimate = interpolate_measurement(first_search, ...
        first_april, first_arm, num_neighbors, new_measurement);
    
    % Check this against ground truth
    error_curr = arm_estimate - second_arm(:, i);
    error(i) = norm(error_curr); 
    
    
end


    
    
    
arm_estimate;