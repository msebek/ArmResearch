function [ah] = PlotSE2( varargin )
% Rudimentary SE2 plotting tool
% TODO: Update to use argin parsing for more flexibility

if nargin < 3
    plotArgs = {'r-'};
else
    plotArgs = varargin{3};
end

if nargin < 2
    fh = figure;
    ah = axes( 'parent', fh );
    se2 = varargin{1};
else
    ah = varargin{1};
    se2 = varargin{2};
end

o = [0;0;1];
x = [0,1;
    0,0;
    1,1];
y = [0,0;
    0,1;
    1,1];
N = numel(se2);

hold( ah, 'on' );
for i = 1:N
    
    H = se2(i).H;
    
    
    xH = H*x;
    yH = H*y;
    
            oH = H*o;
            plot( ah, oH(1,:), oH(2,:), plotArgs{:} );
    
%     plot( ah, xH(1,:), xH(2,:), plotArgs{:} );
%     plot( ah, yH(1,:), yH(2,:), plotArgs{:} );
    
end
