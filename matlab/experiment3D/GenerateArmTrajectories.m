function [points] = GenerateArmTrajectories(points, transitionNoise)
% Takes an input sequence of arm points and generates the body-frame
% displacements for the camera
% For N points, generates N-1 displacement. point(i).displacement takes
% point(i) to point(i+1). The final point has a zero displacement.
%
% Also generates noised versions for filtering testing.
% Returns an augmented struct with fields:
%   armPose - The arm pose in the robot frame
%   cameraPose - The camera pose in the robot frame
%   extrinsics - The camera-arm extrinsics in SE3
%   tagMeasurement - The tag pose in the camera frame
%   trueDisplacement - The true camera body-frame displacements
%   noisedDisplacement - Post-corrupted camera body-frame displacements

N = numel( points );
q = mvnrnd( zeros(1,6), transitionNoise, N-1 )'; % Exponential-coordinate (body velocity) noise

for i = 1:N-1
    points(i).trueDisplacement = points(i).cameraPose\points(i+1).cameraPose;
    points(i).noisedDisplacement = SE3( q(:,i) )*points(i).trueDisplacement;
end

points(N).trueDisplacement = SE3();
points(N).noisedDisplacement = SE3();

