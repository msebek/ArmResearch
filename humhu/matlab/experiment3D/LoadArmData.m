function [data] = LoadArmData( dPath )
% Loads data from a mat file to produce an array of structs with fields:
%   armPose - The arm pose in the robot frame
%   cameraPose - The camera pose in the robot frame
%   extrinsics - The camera-arm extrinsics in SE3
%   tagMeasurement - The tag pose in the camera frame


raw = load( dPath );
N = numel( raw.armState );
arms = cell(1,N);
extrinsics = cell(1,N);
cameras = cell(1,N);
tags = cell(1,N);

armPoses = [ raw.armState.x; raw.armState.y; raw.armState.z; ...
    raw.armState.q1; raw.armState.q2; raw.armState.q3; raw.armState.q0; ];
tagEulers = [ raw.aprilTags.yaw; raw.aprilTags.pitch; raw.aprilTags.roll ];
tagPositions = [raw.aprilTags.x; raw.aprilTags.y; raw.aprilTags.z ];

extrinsicsQuat = euler2quat( [raw.extrinsics.yaw; raw.extrinsics.pitch; raw.extrinsics.roll] );
extPose = [ raw.extrinsics.x; raw.extrinsics.y; raw.extrinsics.z; ...
    extrinsicsQuat ];
ext = SE3( extPose );

% offQuat = euler2quat( [pi/2; pi/2; 0] );
offQuat = euler2quat( [0;0;0] );
atagOffset = SE3( [0; 0; 0; offQuat] );

for i = 1:N
    % Arm Pose SE3 object conversion
    armPoses(4:7,i) = armPoses(4:7,i)/norm(armPoses(4:7,i));
    arms{i} = SE3( armPoses(:,i) );
    
    % Camera-arm extrinsics

    extrinsics{i} = ext;
    
    % April tag detection SE3 objection conversion
    tagQuat = euler2quat( tagEulers(:,i) );
    tagPose = [tagPositions(:,i); tagQuat ];
    tags{i} = SE3( tagPose ) * atagOffset;
    
    cameras{i} = arms{i}*extrinsics{i};
end

data = struct( 'armPose', arms, 'extrinsics', extrinsics, ...
    'cameraPose', cameras, 'tagMeasurement', tags );
