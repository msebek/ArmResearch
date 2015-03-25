classdef LaserScan2D < handle
    
    properties( GetAccess = public, SetAccess = public )
        timestamp;
        angleRange; %[min, max]
        ranges;
        angles;
        points; %[x;y]
    end
    
    methods
        function [obj] = LaserScan2D( ts, angleRanges, ranges )
           
            if nargin == 0
                obj.timestamp = 0;
                obj.angleRange = [0,0];
                obj.ranges = 0;
                obj.angles = 0;
                obj.points = [0;0];
                return;
            end
            
            N = numel(ts);
            nRanges = size( ranges, 1 );
            obj(N) = LaserScan2D;
           
           for i = 1:N
               obj(i).timestamp = ts(i);
               obj(i).angleRange = angleRanges(:,i);
               obj(i).ranges = ranges(:,i);
               obj(i).angles = ...
                   linspace( obj(i).angleRange(1), obj(i).angleRange(2), nRanges );
               
               temp = [ cos(obj(i).angles); ...
                        sin(obj(i).angles) ];
               obj(i).points = bsxfun( @times, temp, obj(i).ranges' );
           end
           
           
        end
    end
    
end