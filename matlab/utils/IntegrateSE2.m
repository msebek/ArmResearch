function [res] = IntegrateSE2( displacements, mode )
T = numel( displacements );
res(T+1) = SE2;

rightMode = strcmp( mode, 'right' );
% Check 'left'

for i = 1:T
    if rightMode
        res(i+1) = res(i)*displacements(i);
    else
        res(i+1) = displacements(i)*res(i);
    end
end