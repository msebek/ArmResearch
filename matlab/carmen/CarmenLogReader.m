classdef CarmenLogReader < handle
    
    properties
        logfile;
    end
    
    methods
   
        function [obj] = CarmenLogReader(filename)
           obj.logfile =  fopen(filename);
        end
        
        function delete(obj)
           fclose(obj.logfile); 
        end
        
        % Returns delimited cell array with specified prefix such that
        % l{1} = prefix
        function [l] = ReadLineType(obj, prefix)
            l = obj.ReadLine();
            while ~isempty(l) && ~strcmp(l.data{1},prefix)
                l = obj.ReadLine();
            end
        end
        
        % Returns struct containing info for next line
        function [line] = ReadLine(obj)
            % If no more lines, return empty
            line = [];
            if ~obj.HasLines()
                return
            end
            
            % Else populate the struct
            rawData = regexp (fgetl(obj.logfile), ' ', 'split');
            
            if strcmp(rawData{1}, '#')
                line = CarmenLogLine({''}, 'Base', [], [], []);
            else
                loggerTimestamp = str2double(rawData(end));
                hostname = rawData(end-1);
                timestamp = str2double(rawData(end-2));
                data = rawData(1:end-3);
                line = CarmenLogLine(data, 'Base', timestamp, hostname, loggerTimestamp);
            end
        end
       
        function [hasMore] = HasLines(obj)
           hasMore = ~feof(obj.logfile);
        end
        
    end
    
end