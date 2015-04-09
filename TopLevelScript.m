% Load all the data files. 
filename = 'arm_april_data_second_set';
load(filename, 'aprilposes', 'armposes');

% Process them. 


% Split them into groups (later)


% 3. Create a KDtree of the april poses to actual position. 
% All observed april poses; rows are observations
% These are the keys (as strings)

%first = aprilposes
%second = 

% Think about different averaging functions

knn_search_object = createns(aprilposes', 'Distance', 'euclidean', ...
        'NSMethod', 'kdtree');

arm_estimate = interpolate_measurement(knn_search_object, ...
        aprilposes, armposes, 3, aprilposes(:, 2));
    
    
    
arm_estimate;