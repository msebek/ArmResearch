classdef SE2
    
    properties(GetAccess = public, SetAccess = private)
        H;
    end
    
    methods
        
        % Takes as input 3x3xN homogeneous transforms, 3xN twist
        % coordinates, or vector [x;y;theta]. Defaults to transforms.
        % SE2( H, 'trans' )
        % SE2( twist, 'twist' )
        % SE2( vec, 'vector' )
        function [obj] = SE2(varargin)
            
            if nargin == 0
                obj.H = eye(3);
                return;
            end
            
            in = varargin{1};
            if nargin < 2
                mode = 'trans';
            else
                mode = varargin{2};
            end
            
            if strcmp( mode, 'vector' )
                N = size(in, 2);
                obj(N) = SE2;
                ct = cos(in(3,:));
                st = sin(in(3,:));
                for i = 1:N
                    obj(i).H = [ ct(i), -st(i), in(1,i);
                        st(i), ct(i), in(2,i);
                        0, 0, 1 ];
                end
                return;
            elseif strcmp( mode, 'trans' )
                N = size(in,3);
                obj(N) = SE2;
                for i = 1:N
                    obj(i).H = in(:,:,i);
                end
            elseif strcmp( mode, 'twist' )
                N = size(in, 2);
                obj(N) = SE2;
                alg = SE2.Algebra(in);
                for i = 1:N
                    obj(i).H = expm( alg(:,:,i) );
                end
            else
                error( 'Invalid input mode' );
            end
        end
        
        function [H] = GetTransform(obj)
            H = reshape( [obj.H], 3, 3, numel(obj) );
        end
        
        % Returns exponential coordinates
        function [v] = GetCoordinates(obj)
            N = numel(obj);
            alg = zeros(3,3,N);
            for i = 1:N
                if any(isinf(obj(i).H(:))) || any(isnan(obj(i).H(:)))
                    blah = 1;
                end
                alg(:,:,i) = logm( obj(i).H );
            end
            v = SE2.Unalgebra(alg);
        end
        
        % Returns [x;y;theta]
        function [v] = GetVector(obj)
            N = numel(obj);
            v = zeros(3,N);
            trans = obj.GetTransform();
            for i = 1:N
                v(1:2,:) = trans(1:2,3,:);
                v(3,:) = atan2( trans(2,1,:), trans(1,1,:) );
            end
        end
        
        function [i] = Inverse(obj)
            N = numel(obj);
            is = zeros(3,3,N);
            for i = 1:N
                is(:,:,i) = inv( obj(i).H );
            end
            i = SE2(is, 'trans');
        end
        
        % obj*b is product obj.H*b.H
        function [p] = mtimes(obj, b)
            N = numel(obj);
            M = numel(b);
            if N ~= M && N ~= 1 && M ~= 1
                error( 'Dimension mismatch.' );
            end
            
            ps = zeros(3,3,max(N,M));
            if N == M
                for i = 1:N
                    ps(:,:,i) = obj(i).H*b(i).H;
                end
            elseif N == 1
                for i = 1:M
                    ps(:,:,i) = obj.H*b(i).H;
                end
            elseif M == 1
                for i = 1:N
                    ps(:,:,i) = obj(i).H*b.H;
                end
            end
            
            p = SE2(ps, 'trans');
        end
        
        % obj/b is product obj.H*inv(b.H)
        function [p] = mrdivide(obj, b)
            N = numel(obj);
            M = numel(b);
            if N ~= M && N ~= 1 && M ~= 1
                error( 'Dimension mismatch.' );
            end
            
            ps = zeros(3,3,max(N,M));
            if N == M
                for i = 1:N
                    ps(:,:,i) = obj(i).H/b(i).H;
                end
            elseif N == 1
                for i = 1:M
                    ps(:,:,i) = obj.H/b(i).H;
                end
            elseif M == 1
                for i = 1:N
                    ps(:,:,i) = obj(i).H/b.H;
                end
            end
            
            p = SE2(ps, 'trans');
        end
        
        % obj\b is product inv(obj.H)*b.H
        function [p] = mldivide(obj, b)
            N = numel(obj);
            M = numel(b);
            if N ~= M && N ~= 1 && M ~= 1
                error( 'Dimension mismatch.' );
            end
            
            ps = zeros(3,3,max(N,M));
            if N == M
                for i = 1:N
                    ps(:,:,i) = obj(i).H\b(i).H;
                end
            elseif N == 1
                for i = 1:M
                    ps(:,:,i) = obj.H\b(i).H;
                end
            elseif M == 1
                for i = 1:N
                    ps(:,:,i) = obj(i).H\b.H;
                end
            end
            
            p = SE2(ps, 'trans');
        end
        
    end
    
    methods(Static)
        
        % Twists are [x;y;th]
        function [alg] = Algebra(coord)
            N = size(coord,2);
            alg = zeros(3,3,N);
            alg(2,1,:) = coord(3,:);
            alg(1,2,:) = -coord(3,:);
            alg(1:2,3,:) = coord(1:2,:);
        end
        
        function [coord] = Unalgebra(alg)
            coord = zeros(3,size(alg,3));
            coord(1,:) = alg(1,3,:);
            coord(2,:) = alg(2,3,:);
            coord(3,:) = alg(2,1,:);
        end
        
    end
    
end