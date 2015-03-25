function [ output_args ] = untitled( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
e = exp(1);
points = [];
for x = 50:5:89.61
    x
    for y = -1412.94:5:-798.5
        for z = 810:5:896
            for i = 1:5
                 q0 = [0.512 0.541 0.527 0.463 0.414];
                 q1 = [0.558 0.529 0.543 0.599 0.632];
                 q2 = [0.446 0.48  0.463 0.388 0.334];
                 q3 = [-0.477 -0.444 -0.461  -0.526  -0.564];
                 q0 = q0(i);
                 q1 = q1(i);
                 q2 = q2(i);
                 q3 = q3(i);
           
%            q0 = 0.455
%            q1 = 0.629
%            q2 = 0.392
%            q3 = -0.494
            x_cor = x;%e^x;
            y_cor = y;%e^y;
            z_cor = z;%e^z;
            points = cat(1,points,[x_cor,y_cor,z_cor,q0,q1,q2,q3]);
        end
    end
    end

end
% for x = 35.15
%     for y = -1200:-1:-1300%:-1375.1
%         for z = 600:10:800
%             for i = 1:5
%                 q0 = [0.512 0.541 0.527 0.463 0.414];
%                 q1 = [0.558 0.529 0.543 0.599 0.632];
%                 q2 = [0.446 0.48  0.463 0.388 0.334];
%                 q3 = [-0.477 -0.444 -0.461  -0.526  -0.564];
%                 q0 = q0(i)
%                 q1 = q1(i)
%                 q2 = q2(i)
%                 q3 = q3(i)
%                         x_cor = x;%e^x;
%                         y_cor = y;%e^y;
%                         z_cor = z;%e^z;
%                        % x_rot_cor = x_rot;%e^x_rot;
%                        % y_rot_cor = y_rot;%e^y_rot;
%                        % z_rot_cor = z_rot;%e^z_rot;
%                       points = cat(1,points,[x_cor,y_cor,z_cor,q0,q1,q2,q3]);
%                end
%             end
%     end
% end
size([-log(-1.5):.01:log(1.5)])
howLongPerLocation = 5
howLongwillItTakeSeconds = size(points,1)*5
howLongwillItTakeMinutes = howLongwillItTakeSeconds/60
howLongwillItTakeHours   = howLongwillItTakeMinutes/60
csvwrite('points.csv',points)
end

