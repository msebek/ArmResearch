classdef CarmenLogLaserReader < CarmenLogReader
    
    properties
        laserPrefix;
        angleStart;
        angleEnd;
    end
    
    methods
        
        function [obj] = CarmenLogLaserReader(filename, prefix, angleRange)
            obj = obj@CarmenLogReader(filename);
            obj.laserPrefix = prefix;
            obj.angleStart = angleRange(1);
            obj.angleEnd = angleRange(2);
        end
        
        function [scan] = ReadScan(obj)
           % If no more lines, return
           scan = [];
           if ~obj.HasLines()
               return;
           end
           
           line = obj.ReadLineType(obj.laserPrefix);
           numPoints = str2double(line.data{2});
           ranges = zeros(1, numPoints);
           angles = zeros(1, numPoints);
           angleStep = (obj.angleEnd - obj.angleStart)/numPoints;
           for i = 1:numPoints
               ranges(i) = str2double(line.data{2 + i});
               angles(i) = obj.angleStart + (i-1)*angleStep;
           end
           laserData = LaserScan(ranges, angles);
           scan = CarmenLogLine(laserData, 'LaserData', line.timestamp, ...
               line.hostname, line.loggerTimestamp);
               
        end
        
    end
    
end