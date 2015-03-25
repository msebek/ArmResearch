classdef ExtendedKalmanFilter < handle
    
    properties( GetAccess=public, SetAccess=private )
        
        f; % State transition function f(x, u)
        F; % State transition Jacobian F(x, u)
        
        h; % Observation function h(x)
        H; % Observation Jacobian H(x)
        
        hDiff; % Observation differencing function
        
        x; % Current state
        S; % Current estimate covariance
        
        Qset; % Transition covariance, if specified
        Rset; % Observation covariance, if specified
    end
    
    methods
       
        function [obj] = ExtendedKalmanFilter(f, F, h, H, hDiff)
           if nargin == 0
               return;
           end
           
           if nargin < 5
               hDiff = @(a,b) a - b;
           end
           
           obj.f = f;
           obj.F = F;
           
           obj.h = h;
           obj.H = H;
           obj.hDiff = hDiff;
           
           obj.Qset = [];
           obj.Rset = [];
           
        end
        
        function Initialize(obj, x0, S0)
        
            obj.x = x0;
            obj.S = S0;
            
        end
        
        function SetTransitionCovariance(obj, Q)
           obj.Qset = Q; 
        end
        
        function SetObservationFunctions( obj, h, H )
            obj.h = h;
            obj.H = H;
        end
        
        function SetObservationCovariance(obj, R)
            obj.Rset = R;
        end
        
        function [x] = Predict(obj, u, Q)
            
            if nargin < 3
                Q = obj.Qset;
            end
            
            Fi = obj.F(obj.x, u);
            x = obj.f(obj.x, u);
            obj.x = x;
            obj.S = Fi*obj.S*Fi' + Q;
            
        end
        
        function [innovation, C, residual, Cv] = Update(obj, z, R)
        
            if nargin < 3
                R = obj.Rset;
            end
            
            Hi = obj.H(obj.x);
            C = Hi*obj.S*Hi';
            K = obj.S*Hi'/(C+R);
            innovation = obj.hDiff(z, obj.h(obj.x));
            obj.x = obj.x + K*innovation;
            obj.S = obj.S - K*Hi*obj.S;
            
            % Extra residual calculations
            residual = obj.hDiff(z, obj.h(obj.x));
            Hi = obj.H(obj.x);
            Cv = Hi*obj.S*Hi';
            
        end
            
    end
    
end