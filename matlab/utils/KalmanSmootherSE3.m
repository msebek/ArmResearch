classdef KalmanSmootherSE3 < handle
    % Tracks a SE3 pose using the vector space to update
    % Noise can be applied pre or post:
    %   leftsample = SE3( mvnrnd( zeros(1,6), S ) ) * mean;
    %   rightsample = mean * SE3( mvnrnd( zeros(1,6), S ) );
    % Depending on the transform convention, this can be noise at the start or
    % the end of the kinematic chain.
    
    properties(GetAccess = public, SetAccess = private)
        x; % The estimate mean in exponential coordinates
        S; % The estimate covariance
        leftMode; % Bool setting left multiply noise mode
        Qset;
        Rset;
        
        prioriX; % Previous priori means
        prioriS; % Previous priori covariances
        posterioriX; % Previous posteriori means
        posterioriS; % Previous posteriori covariances
    end
    
    methods
        
        % Modes:
        % KFSE3( x0, S0, mode )
        % KFSE3( x0, S0 ) [default mode = left]
        % KFSE3( mode ) [default x0 = 0, S0 = I]
        % KFSE3() [defaults above]
        % Note - does not support array construction
        function [obj] = KalmanSmootherSE3( varargin )
            
            if nargin == 0
                x0 = SE3();
                S0 = eye(6);
                m = 'left';
            elseif nargin == 1
                x0 = SE3();
                S0 = eye(6);
                m = varargin{1};
            elseif nargin == 2
                x0 = varargin{1};
                S0 = varargin{2};
                m = 'left';
            elseif nargin == 3
                x0 = varargin{1};
                S0 = varargin{2};
                m = varargin{3};
            else
                error( 'Invalid number of arguments' );
            end
            
            KalmanSmootherSE3.CheckX( x0 );
            KalmanSmootherSE3.CheckS( S0 );
            KalmanSmootherSE3.CheckMode( m );
            
            obj.x = x0;
            obj.S = S0;
            obj.leftMode = strcmp( m, 'left' );
            obj.Qset = [];
            obj.Rset = [];
            
        end
        
        function [] = Initialize( obj, x0, S0 )
            KalmanSmootherSE3.CheckX( x0 );
            KalmanSmootherSE3.CheckS( S0 );
            obj.x = x0;
            obj.S = S0;
            
            obj.prioriX = x0;
            obj.prioriS = S0;
        end
        
        function SetTransitionCovariance( obj, Q )
            obj.Qset = Q;
        end
        
        function SetObservationCovariance( obj, R )
            obj.Rset = R;
        end
        
        function [] = SetMode( obj, mode )
            KalmanSmootherSE3.CheckMode( mode );
            m = strcmp( mode, 'left' );
            if m == obj.leftMode
                return
            end
            
            % Going from right to left
            if m
                A = obj.x.GetAdjoint();
            else
                A = inv(obj.x.GetAdjoint());
            end
            obj.S = A*obj.S*A';
            obj.leftMode = m;
        end
        
        % Propogates the estimate according to a displacement and its
        % uncertainty. Can have different conventions than the estimate
        % noise mode.
        % Modes:
        % Predict( d ) - set noise, noise left
        % Predict( d, Q ) - displacement right, noise left
        % Predict( d, Q, mode ) - displacement and noise same mode
        % Predict (d, dMode, Q, QMode )
        function [] = Predict( obj, varargin )
            
            if numel( varargin ) == 0
                Q = obj.Qset;
                d = SE3();
                dM = 'right';
                QM = 'left';
            elseif numel( varargin ) == 1
                Q = obj.Qset;
                d = varargin{1};
                dM = 'right';
                QM = 'left';
            elseif numel( varargin ) == 2
                d = varargin{1};
                Q = varargin{2};
                dM = 'right';
                QM = 'left';
            elseif numel( varargin ) == 3
                d = varargin{1};
                Q = varargin{2};
                dM = varargin{3};
                QM = varargin{3};
            elseif numel( varargin ) == 4
                d = varargin{1};
                dM = varargin{2};
                Q = varargin{3};
                QM = varargin{4};
            else
                error( 'Invalid number of arguments.' );
            end
            
            KalmanSmootherSE3.CheckX( d );
            KalmanSmootherSE3.CheckS( Q );
            KalmanSmootherSE3.CheckMode( dM );
            KalmanSmootherSE3.CheckMode( QM );
            
            dMode = strcmp( dM, 'left' );
            QMode = strcmp( QM, 'left' );
            
            if obj.leftMode && dMode && QMode
                obj.x = d*obj.x;
                Ad = d.GetAdjoint();
                obj.S = Q + Ad*obj.S*Ad';
            elseif obj.leftMode && dMode && ~QMode
                obj.x = d*obj.x;
                Ad = d.GetAdjoint();
                obj.S = Ad*(Q+obj.S)*Ad';
            elseif obj.leftMode && ~dMode && QMode
                Ax = obj.x.GetAdjoint();
                obj.x = obj.x*d;
                obj.S = obj.S + Ax*Q*Ax';
            elseif obj.leftMode && ~dMode && ~QMode
                obj.x = obj.x*d;
                Axd = obj.x.GetAdjoint();
                obj.S = obj.S + Axd*Q*Axd';
            elseif ~obj.leftMode && dMode && QMode
                obj.x = d*obj.x;
                Adx = inv(obj.x.GetAdjoint());
                obj.S = obj.S + Adx*Q*Adx';
            elseif ~obj.leftMode && dMode && ~QMode
                Ax = inv(obj.x.GetAdjoint());
                obj.x = d*obj.x;
                obj.S = obj.S + Ax*Q*Ax';
            elseif ~obj.leftMode && ~dMode && QMode
                Ad = inv(d.GetAdjoint());
                obj.x = obj.x*d;
                obj.S = Ad*(obj.S + Q)*Ad';
            elseif ~obj.leftMode && ~dMode && ~QMode
                Ad = inv(d.GetAdjoint());
                obj.x = obj.x*d;
                obj.S = Q + Ad*obj.S*Ad';
            end
            
            obj.prioriX(end+1) = obj.x;
            obj.prioriS(:,:,end+1) = obj.S;
            
        end
        
        % Updates using a full pose observation
        % Modes
        %   Update( z, R ) [default Rmode = left]
        %   Update( z, R, RMode )
        function [innovation] = Update( obj, varargin )
            
            if numel( varargin ) == 1
                z = varargin{1};
                R = obj.Rset;
                RM = 'left';
            elseif numel( varargin ) == 2
                z = varargin{1};
                R = varargin{2};
                RM = 'left';
            else
                z = varargin{1};
                R = varargin{2};
                RM = varargin{3};
            end
            
            KalmanSmootherSE3.CheckX(z);
            KalmanSmootherSE3.CheckS(R);
            KalmanSmootherSE3.CheckMode(RM);
            
            % Update must be performed with noise samples on left, or more
            % specifically, uncertainty at end of the chain
            RMode = strcmp( RM, 'left' );
            if ~RMode
                Az = z.GetAdjoint();
                R = Az*R*Az';
            end
            
            xt = obj.x;
            St = obj.S;
            if ~obj.leftMode
                Ax = xt.GetAdjoint();
                St = Ax*St*Ax';
            end
            
            K = St/( St + R );
            
            poseErr = z/xt;
            innovation = poseErr.GetCoordinates();
            
            xt = SE3( K*innovation )*xt;
            St = ( eye(6) - K)*St;
            
            if ~obj.leftMode
                Axinv = inv(xt.GetAdjoint());
                St = Axinv*St*Axinv';
            end
            
            obj.x = xt;
            obj.S = St;
            
            if isempty( obj.posterioriX )
                obj.posterioriX = obj.x;
            else
                obj.posterioriX(end+1) = obj.x;
            end
            if isempty( obj.posterioriS )
                obj.posterioriS = obj.S;
            else
                obj.posterioriS(:,:,end+1) = obj.S;
            end
            
        end
        
        function [sX, sS] = SmoothAll(obj)
           
            T = numel( obj.posterioriX );
            
            sX = SE3.empty(1,0);
            sS = zeros(6,6,T);
            sX(T) = obj.posterioriX(T);
            sS(:,:,T) = obj.posterioriS(:,:,T);
            for t = T-1:-1:1
                
                G = obj.posterioriS(:,:,t)/obj.prioriS(:,:,t+1);
                
                %sX(:,t) = obj.posterioriX(:,t) + G*( obj.fDiff(sX(:,t+1), obj.prioriX(:,t+1)) );
                xErr = sX(:,t+1)/obj.prioriX(t+1);
                sX(:,t) = SE3( G*xErr.GetCoordinates() ) * obj.posterioriX(:,t);
                
                sS(:,:,t) = obj.posterioriS(:,:,t) + G*( sS(:,:,t+1) - obj.prioriS(:,:,t+1) )*G';
            end
            
        end
        
    end
    
    methods( Static, Access = private )
        function [] = CheckX( x )
            if ~isa( x, 'SE3' )
                error( 'Pose must be an SE3 object' );
            end
        end
        
        function [] = CheckS( S )
            if ~isa( S, 'double' ) || ...
                    any( size(S) ~= [6,6] ) || ...
                    any( eig(S) < eps )
                error( 'Covariance must be a 6 x 6 PD matrix' );
            end
        end
        
        function [] = CheckMode( m )
            if ~isa( m, 'char' ) || ...
                    ( ~strcmp( m, 'left' ) && ~strcmp( m, 'right' ) )
                error( 'Noise mode must be a left or right' );
            end
        end
        
    end
    
end