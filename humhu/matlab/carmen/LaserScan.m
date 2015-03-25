classdef LaserScan < handle
    
    properties
        ranges;
        angles;
    end
   
    methods
       
        function [obj] = LaserScan(rng, ang)
            obj.ranges = rng(:);
            obj.angles = ang(:);
        end
        
        function [points] = ToCloud(obj)
           
            trigs = [cos(obj.angles), sin(obj.angles)];
            points = bsxfun(@times, obj.ranges, trigs);
            
        end
        
    end
    
end