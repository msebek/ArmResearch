function [ah] = PlotSE3( varargin )

if nargin < 3
    plotMode = 'default';
else
    plotMode = 'custom';
    plotArgs = varargin{3};
end

if nargin < 2
    fh = figure;
    ah = axes( 'parent', fh );
    se3 = varargin{1};
else
    ah = varargin{1};
    se3 = varargin{2};
end

o = [0;0;0;1];
x = [0,0.1;
     0,0;
     0,0;
     1,1];
y = [0,0;
     0,0.1;
     0,0;
     1,1];
z = [0,0;
     0,0;
     0,0.1;
     1,1];

N = numel(se3);

hold( ah, 'on' );
for i = 1:N
    
    H = se3(i).H;
    if strcmp( plotMode, 'default' )
    
    xH = H*x;
    yH = H*y;
    zH = H*z;
    
    plot3( ah, xH(1,:), xH(2,:), xH(3,:), 'r-' );
    plot3( ah, yH(1,:), yH(2,:), yH(3,:), 'g-' );
    plot3( ah, zH(1,:), zH(2,:), zH(3,:), 'b-' );
    elseif strcmp( plotMode, 'custom' )
       oH = H*o; 
       plot3( ah, oH(1,:), oH(2,:), oH(3,:), plotArgs{:} );
    end
    
end

axis( ah, 'vis3d', 'equal' );
xlabel( ah, 'x' );
ylabel( ah, 'y' );
zlabel( ah, 'z' );
