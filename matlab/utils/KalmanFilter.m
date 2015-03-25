classdef KalmanFilter < handle
    
    properties
        
        A; % State transition
        B; % Input transition mapping
        Q; % State transition noise
        C; % Observation matrix
        R; % Observation noise
        
        x; % Current state
        S; % Current estimate covariance
        
    end
    
    methods
       
        function [obj] = KalmanFilter(A, B, Q, C, R)
           if nargin == 0
               return;
           end
           obj.A = A;
           obj.B = B;
           obj.Q = Q;
           obj.C = C;
           obj.R = R;
        end
        
        function Initialize(obj, x0, S0)
        
            obj.x = x0;
            obj.S = S0;
            
        end
        
        function [x] = Predict(obj, u, q)
           
            if nargin < 3
               q = obj.Q; 
            end
            
            if nargin < 2
                obj.x = obj.A*obj.x;
            else
                obj.x = obj.A*obj.x + obj.B*u;
            end
            
            x = obj.x;
            obj.S = obj.A*obj.S*obj.A' + q;
            
        end
        
        function [x] = Update(obj, z, r)
        
            if nargin < 3
               r = obj.R; 
            end
            
            K = obj.S*obj.C'/(obj.C*obj.S*obj.C' + r);
            obj.x = obj.x + K*(z - obj.C*obj.x);
            obj.S = obj.S - K*obj.C*obj.S;
            x = obj.x;
            
        end
            
    end
end