classdef CarmenLogLine
    
    properties(GetAccess = public, SetAccess = immutable)
        data;   % Depends on line type
        datatype;
        timestamp;
        hostname;
        loggerTimestamp;
    end
    
    methods
        
        function [obj] = CarmenLogLine(dat, dattype, tstamp, hname, ltstamp)
            
            obj.data = dat;
            obj.datatype = dattype;
            
            obj.timestamp = tstamp;
            obj.hostname = hname;
            obj.loggerTimestamp = ltstamp;
            
        end
        
    end
    
end


