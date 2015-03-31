function [ H ] = my_rpytohomog ( input )
%
% input = 6x1 of [x; y; z; pitch; roll; yaw]
    H = zeros(4, 4);
    
    x = input(1);
    y = input(2);
    z = input(3);
    pitch = input(4);
    roll = input(5);
    yaw = input(6);
    
    yaw = [cos(yaw), -sin(yaw), 0; 
           sin(yaw), cos(yaw), 0; 
                         0,              0,  1];
                     
    pitch = [cos(pitch), 0, sin(pitch);
                            0,  1, 0;
             -sin(pitch), 0, cos(pitch)];
         
    roll =[ 1,    0,    0;
            0, cos(roll), -sin(roll);
            0, sin(roll), cos(roll)];
         
    arm_rot = yaw * pitch * roll;
    
    H(1:3, 1:3) = arm_rot;
    H(1:3, 4) = [x, y, z]';
    H(4, 4) = 1;
    

end