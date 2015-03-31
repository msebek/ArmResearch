function [ H ] = my_quattohomog ( input )
%
% input = 6x1 of [x; y; z; q0; q1; q2; q3]
    H = zeros(4, 4);
    
    x = input(1);
    y = input(2);
    z = input(3);    
    
    % quat2mat takes x, y, z, w
    H(1:3, 1:3) = quat2mat([input(5:7); input(4)]);
    H(1:3, 4) = [x, y, z]';
    H(4, 4) = 1;
    
end