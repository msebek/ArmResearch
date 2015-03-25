function [q] = mat2quat(mat)

N = size(mat,3);
q = zeros(4,N);
for i = 1:N
    m = mat(:,:,i);
    t = trace(m);
    
    if( any(diag(m) == 1) )
        ai = find(diag(m) == 1,1,'first');
        aor = false(size(m,1),1);
        aor(ai) = true(1);
        R = m(~aor, ~aor);
        ang = atan2(R(2,1), R(1,1));
        if ai == 2
            ang = -ang; % Have to flip pitch here
        end
        qt = zeros(4,1);
        qt(4) = cos(ang/2);
        qt(find(aor)) = sin(ang/2);
        q(:,i) = qt;
        continue;
    end
    
    Qxx = m(1,1);
    Qyy = m(2,2);
    Qzz = m(3,3);
    Qzy = m(3,2);
    Qyz = m(2,3);
    Qxz = m(1,3);
    Qzx = m(3,1);
    Qyx = m(2,1);
    Qxy = m(1,2);
    
    if t > 0
        r = sqrt(1+t);
        w = 0.5*r;
        x = copysign(0.5*sqrt(1+Qxx-Qyy-Qzz), Qzy-Qyz);
        y = copysign(0.5*sqrt(1-Qxx+Qyy-Qzz), Qxz-Qzx);
        z = copysign(0.5*sqrt(1-Qxx-Qyy+Qzz), Qyx-Qxy);
        
    else
        r = sqrt(1+Qxx-Qyy-Qzz);
        s = 0.5/r;
        w = (Qzy-Qyz)*s;
        x = 0.5*r;
        y = (Qxy+Qyx)*s;
        z = (Qzx+Qxz)*s;
    end
    
    
    q(:,i) = -[x; y; z; w];
end

function [cs] = copysign(x, y)
cs = sign(y)*abs(x);
