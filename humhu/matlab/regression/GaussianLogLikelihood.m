function [ll] = GaussianLogLikelihood( x, X, mode )
% Calculates the log likelihood of samples x assuming a zero-mean Gaussian
% with:
%   covariances X if mode = 'covariance'
%   informations X if mode = 'information'

N = size(x,2);

dets = zeros(N,1);
quads = zeros(N,1);

if strcmp( mode, 'covariance' ) || strcmp( mode, 'fullcovariance' )
    covMode = true;
elseif strcmp( mode, 'information' )
    covMode = false;
else
	error('Must specify log likelihood mode.');
end

for i = 1:N
    if covMode
%         M = inv(X(:,:,i));
        M = X(:,:,i);
        d = det(M);
        r = rcond( M );
        
        % Catch all sorts of things here
        if d == 0 || r < eps || isnan( r )
           dets(i) = 0;
           quads(i) = Inf; % Forces ll to -Inf
        else
            dets(i) = 1/d;
            quads(i) = (x(:,i)'/M)*x(:,i);
        end
    else
        M = X(:,:,i);
        dets(i) = det(M);
        quads(i) = x(:,i)'*M*x(:,i);
    end
end

ll = 0.5*log(dets) -0.5*quads;
k = size(x,1);
ll = ll - k/2*log(2*pi);