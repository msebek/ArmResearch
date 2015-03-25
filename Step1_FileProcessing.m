% Process all the file triples in the input path
% Output: 
%  april_poses = [ april_id, april_hamming, april_distance,
%                        april_x, april_y, april_z, 
%                        apr_yaw, apr_pitch, apr_roll ];
%  
%  arm_poses = [ apr_x, apr_y, apr_z, 
%                   apr_q0, apr_q1, apr_q2, apr_q3, 
%                   apr_q4 ]


%input_path = 'D:\moveing\';
input_path = 'dataSet2\';

all_files = dir(input_path);
% Remove all the .txt files from our search


file_prefixes = {};
for i = 3:size(all_files, 1)
   if(numel(all_files(i).name) < 12)
       continue
   end
   if(all_files(i).name(end-11:end) == 'armState.txt')
       file_prefixes{end+1} = all_files(i).name(1:end-12);
   end
end

armposes = [];
aprilposes = [];
%% Open the files, and read them in. 
for pre=file_prefixes
    % Open the True positions
    april_pose_name = strcat(input_path, strcat(pre,'.txt'));
    arm_pose_name = strcat(input_path, strcat(pre, 'armState.txt'));
    
    try
        aprilposes = [aprilposes, dlmread(april_pose_name{1}, ',', 0, 1)];
        armposes = [armposes, dlmread(arm_pose_name{1}, ',', 0, 1)];
    catch ME
        switch ME.identifier
            case 'MATLAB:textscan:BuildFormatString'
                continue;
            otherwise
                rethrow(ME);
        end
                
    end
end

save('arm_april_data', 'aprilposes', 'armposes');

