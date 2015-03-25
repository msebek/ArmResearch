classdef WindowedCovarianceEstimator < handle
    
    properties( GetAccess=public, SetAccess=private )
        windowSize;
        residuals;
        R;
    end
    
    methods
        
        function [obj] = WindowedCovarianceEstimator( zDim, wSize )
            
            if nargin == 0
                obj.windowSize = 10;
                zDim = 2;
            else
                obj.windowSize = wSize;
            end
            obj.R = eye(zDim);
        end
        
        function Initialize( obj, R0 )
            obj.R = R0;
        end
        
        % Updates with one sample at a time
        % Takes a new residual sample and the mapped residual covariance
        function Update( obj, residual, mCov )
            
            if isempty( obj.residuals )
                obj.residuals = residual;
            else
                obj.residuals(:,end+1) = residual;
            end
            
            % Pare down the window
            if size(obj.residuals,2) > obj.windowSize
                obj.residuals = obj.residuals(:,(end-obj.windowSize+1):end);
            end
            
            % Only calculate if window is full
            if size(obj.residuals,2) == obj.windowSize
                Cv = cov( obj.residuals' );
                obj.R = diag( diag(Cv + mCov) );
            end
            
        end
        
        function [R] = GetCovariance( obj )
            R = obj.R;
        end
        
    end
    
end