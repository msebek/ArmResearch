classdef SE3
    
    properties(GetAccess = public, SetAccess = private)
        H; % Homogeneous matrix representation
    end
    
    methods
        
        % Takes as input 4x4xN homogeneous transforms, 6xN twist
        % coordinates, or 7xN [pos;quat]
        function [obj] = SE3(argin)
            
            if nargin == 0
                obj.H = eye(4);
                return;
            end
            
            if size(argin,1) == 4
                N = size(argin,3);
                obj(N) = SE3;
                for i = 1:N
                    obj(i).H = argin(:,:,i);
                end
            elseif size(argin,1) == 6
                N = size(argin,2);
                obj(N) = SE3;
                alg = SE3.Algebra(argin);
                for i = 1:N
                    obj(i).H = expm( alg(:,:,i) );
                end
            elseif size(argin,1) == 7
                N = size(argin,2);
                obj(N) = SE3;
                for i = 1:N
                    pos = argin(1:3,i);
                    quat = argin(4:7,i);
                    R = eye(4);
                    R(1:3,1:3) = quat2mat( quat );
                    T = eye(4);
                    T(1:3,4) = pos;
                    obj(i).H = T*R;
                end
            else
                error( 'Input dimension must be 4 or 6' );
            end
        end
        
        function [H] = GetTransform(obj)
            H = reshape( [obj.H], 4, 4, numel(obj) );
        end
        
        function [v] = GetVector(obj)
            N = numel(obj);
            v = zeros(7,N);
            for i = 1:N
                pos = obj(i).H(1:3,4);
                quat = mat2quat(obj(i).H(1:3,1:3));
                v(:,i) = [pos; quat];
            end
        end
        
        function [c] = GetCoordinates(obj)
            N = numel(obj);
            alg = zeros(4,4,N);
            for i = 1:N
                alg(:,:,i) = logm( obj(i).H );
            end
            c = SE3.Unalgebra(alg);
        end
        
        function [i] = Inverse(obj)
            N = numel(obj);
            is = zeros(4,4,N);
            for i = 1:N
                is(:,:,i) = inv( obj(i).H );
            end
            i = SE3(is);
        end
        
        % obj*b is product obj.H*b.H
        function [p] = mtimes(obj, b)
            N = numel(obj);
            M = numel(b);
            if N ~= M && N ~= 1 && M ~= 1
                error( 'Dimension mismatch.' );
            end
            
            ps = zeros(4,4,max(N,M));
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
            
            p = SE3(ps);
        end
        
        % obj/b is product obj.H*inv(b.H)
        function [p] = mrdivide(obj, b)
            N = numel(obj);
            M = numel(b);
            if N ~= M && N ~= 1 && M ~= 1
                error( 'Dimension mismatch.' );
            end
            
            ps = zeros(4,4,max(N,M));
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
            
            p = SE3(ps);
        end
        
        % obj\b is product inv(obj.H)*b.H
        function [p] = mldivide(obj, b)
            N = numel(obj);
            M = numel(b);
            if N ~= M && N ~= 1 && M ~= 1
                error( 'Dimension mismatch.' );
            end
            
            ps = zeros(4,4,max(N,M));
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
            
            p = SE3(ps);
        end
        
        function [A] = GetAdjoint(obj)
            N = numel(obj);
            A = zeros(6,6,N);
            for i = 1:N
                R = obj.H(1:3,1:3,i);
                tx = SE3.CrossMatrix( obj.H(1:3,4) );
                A(:,:,i) = [ R, tx*R;
                    zeros(3,3), R ];
            end
        end
        
    end
    
    methods(Static)
        
        % Coordinates are [u;omega]
        function [alg] = Algebra(coord)
            N = size(coord,2);
            alg = zeros(4,4,N);
            alg(1:3,1:3,:) = SE3.CrossMatrix(coord(4:6,:));
            alg(1:3,4,:) = coord(1:3,:);
        end
        
        function [coord] = Unalgebra(alg)
            coord = zeros(6,size(alg,3));
            coord(1:3,:) = squeeze(alg(1:3,4,:));
            coord(4:6,:) = SE3.UncrossMatrix(alg(1:3,1:3,:));
        end
        
        function [m] = CrossMatrix(v)
            m = zeros(3,3,size(v,2));
            m(2,1,:) = v(3,:);
            m(1,2,:) = -v(3,:);
            m(3,1,:) = -v(2,:);
            m(1,3,:) = v(2,:);
            m(3,2,:) = v(1,:);
            m(2,3,:) = -v(1,:);
        end
        
        function [v] = UncrossMatrix(m)
            v = zeros(3,size(m,3));
            v(1,:) = squeeze(m(3,2,:));
            v(2,:) = squeeze(m(1,3,:));
            v(3,:) = squeeze(m(2,1,:));
        end
    end
    
end