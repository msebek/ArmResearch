function [n] = vnorm( V )

N = size(V,2);
n = zeros(1,N);
for i = 1:N
   n(i) = norm(V(:,i)); 
end