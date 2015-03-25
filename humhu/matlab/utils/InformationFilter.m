classdef InformationFilter < handle
    
    properties
        
        A; % State transition
        B; % Input transition mapping
        Q; % State transition noise
        C; % Observation matrix
        Rinv; % Inverse observation noise
        
        y; % Current information state
        Y; % Current estimate information
        
    end
    
    methods
       
        function [obj] = InformationFilter(A, B, Q, C, Rinv)
           if nargin == 0
               return;
           end
           obj.A = A;
           obj.B = B;
           obj.Q = Q;
           obj.C = C;
           obj.Rinv = Rinv;
        end
        
        function Initialize(obj, x0, S0)
        
            obj.SetState( x0, S0 );
            
        end
        
        function [] = Predict(obj, u, q)
           
            if nargin < 3
               q = obj.Q; 
            end
            
            [x, S] = GetState(obj);
            
            if nargin < 2
                x = obj.A*x;
            else
                x = obj.A*x + obj.B*u;
            end
            
            S = obj.A*S*obj.A' + q;
            
            obj.SetState( x, S );
            
        end
        
        function [] = Update(obj, z, Rinv)
        
            if nargin < 3
               Rinv = obj.Rinv; 
            end
            
            I = obj.C'*Rinv*obj.C;
            i = obj.C'*Rinv*z;
            obj.Y = obj.Y + I;
            obj.y = obj.y + i;
            
        end
        
        function [x,S] = GetState(obj)
            x = obj.Y\obj.y;
            S = inv(obj.Y);
        end
            
        function [] = SetState(obj, x, S)
            obj.Y = inv(S);
            obj.y = obj.Y*x;
        end
        
    end
end