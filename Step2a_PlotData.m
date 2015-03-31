clear all;
close all;
filename = 'arm_april_data_third_set';

load(filename, 'aprilposes', 'armposes');

% Get arm_poses x,y,z
scatter3(armposes(1, :), armposes(2, :), armposes(3, :));
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');



% During lookup, find the twist between april measurement, and the other
% april measurements. Mahalanobis?

% Convert RPY to components (u, v, w)
%u = armposes(4,:);
%v = armposes(5,:);
%w = armposes(6,:);
%quiver3(aprilposes(1,:),armposes(2,:),armposes(3,:),u,v,w);

% Error in XYZ of data
%arm_xyz = armposes(1:3, :);
%april_xyz = aprilposes(3:5, :);

%error = arm_xyz - april_xyz;
