classdef ExtendedKalmanSmoother < handle
    
    properties( GetAccess=public, SetAccess=private )
        
        f; % State transition function f(x, u)
        F; % State transition Jacobian F(x, u)
        
        h; % Observation function h(x)
        H; % Observation Jacobian H(x)
        
        fDiff; % State differencing function
        hDiff; % Observation differencing function
        
        x; % Current state
        S; % Current estimate covariance
        
        Qset; % Transition covariance, if specified
        Rset; % Observation covariance, if specified
        
        FHist;
        prioriX; % Previous priori means
        prioriS; % Previous priori covariances
        posterioriX; % Previous posteriori means
        posterioriS; % Previous posteriori covariances
        
    end
    
    methods
        
        function [obj] = ExtendedKalmanSmoother(f, F, h, H, fDiff, hDiff)
            if nargin == 0
                return;
            end
            
            if nargin < 5
                hDiff = @(a,b) a - b;
            end
            
            obj.f = f;
            obj.F = F;
            obj.fDiff = fDiff;
            
            obj.h = h;
            obj.H = H;
            obj.hDiff = hDiff;
            
            obj.Qset = [];
            obj.Rset = [];
            
        end
        
        function Initialize(obj, x0, S0)
            
            obj.x = x0;
            obj.S = S0;
            
            obj.prioriX = x0;
            obj.prioriS = S0;
            
        end
        
        function SetTransitionCovariance(obj, Q)
            obj.Qset = Q;
        end
        
        function SetObservationCovariance(obj, R)
            obj.Rset = R;
        end
        
        function SetObservationFunctions(obj, h, H)
            obj.h = h;
            obj.H = H;
        end
        
        function [x] = ForwardPredict(obj, u, Q)
            
            if nargin < 3
                Q = obj.Qset;
            end
            
            Fi = obj.F(obj.x,u);
            x = obj.f(obj.x, u);
            obj.x = x;
            obj.S = Fi*obj.S*Fi' + Q;
            
            if isempty( obj.FHist )
                obj.FHist = Fi;
            end
            obj.FHist(:,:,end+1) = Fi;
            obj.prioriX(:,end+1) = obj.x;
            obj.prioriS(:,:,end+1) = obj.S;
        end
        
        function [innovation, C] = ForwardUpdate(obj, z, R)
            
            if nargin < 3
                R = obj.Rset;
            end
            
            Hi = obj.H(obj.x);
            C = Hi*obj.S*Hi';
            K = obj.S*Hi'/(C+R);
            innovation = obj.hDiff(z, obj.h(obj.x));
            obj.x = obj.x + K*innovation;
            obj.S = obj.S - K*Hi*obj.S;
        end
        
        function [] = ClampPosteriori( obj )
            
            if isempty( obj.posterioriX )
                obj.posterioriX = obj.x;
            else
                obj.posterioriX(:,end+1) = obj.x;
            end
            if isempty( obj.posterioriS )
                obj.posterioriS = obj.S;
            else
                obj.posterioriS(:,:,end+1) = obj.S;
            end
        end
        
        % Runs smoothing on the saved trajectory
        function [sX, sS] = SmoothAll(obj)
            
            xDim = size( obj.x, 1 );
            T = size( obj.posterioriX, 2 );
            
            sX = zeros(xDim,T);
            sS = zeros(xDim,xDim,T);
            sX(:,T) = obj.posterioriX(:,T);
            sS(:,:,T) = obj.posterioriS(:,:,T);
            for t = T-1:-1:1
                G = obj.posterioriS(:,:,t)*obj.FHist(:,:,t+1)'/obj.prioriS(:,:,t+1);
                sX(:,t) = obj.posterioriX(:,t) + G*( obj.fDiff(sX(:,t+1), obj.prioriX(:,t+1)) );
                sS(:,:,t) = obj.posterioriS(:,:,t) + G*( sS(:,:,t+1) - obj.prioriS(:,:,t+1) )*G';
            end
            
        end
        
    end
    
end