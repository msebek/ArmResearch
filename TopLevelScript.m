% Load all the data files. 
filename = 'arm_april_data_first_set';
load(filename, 'aprilposes', 'armposes');

% Process them. 


% Split them into groups (later)


% 3. Create a KDtree of the april poses to actual position. 
% All observed april poses; rows are observations
% These are the keys (as strings)
knn_search_object = createns(aprilposes', 'Distance', 'euclidean', ...
        'NSMethod', 'kdtree');

