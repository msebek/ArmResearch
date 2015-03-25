function [dW] = PositiveBasisGradient( x, f, C, model )
% Calculates the gradient for the positive basis regressor form

nf = size(f,1);
nBasis = size(model.basis,3);
nData = size(f,2);
nx = size(model.basis,1);

[S, w] = PositiveBasisRegression( f, model );
S = S + C;

% Scale features after regression for gradient calculations
f = bsxfun( @rdivide, f, model.featureScales );

Sinv = zeros( size(S) );
for i = 1:nData
    Sinv(:,:,i) = inv(S(:,:,i));
end

dW = zeros( nf, nBasis );

if strcmp( model.modelMode, 'covariance' )
    for k = 1:nData
        
        Sinvk = Sinv(:,:,k);
        xk = x(:,k);
        fk = f(:,k);
        
        iSinvkouter = -eye(nx) + Sinvk*(xk*xk');
        
        dWi = zeros(1,nBasis);
        for i = 1:nBasis
            % dWi(i) = trace( -Sinvk*Bi + Sinvk*Bi*Sinvk*xk*xk' )*wi;
            dWi(i) = trace( Sinvk*model.basis(:,:,i)*iSinvkouter )*w(i,k);
        end
        dW = dW + bsxfun(@times, dWi, fk);
        
    end
elseif strcmp( model.modelMode, 'information' )
    for k = 1:nData
        
        Sinvk = Sinv(:,:,k);
        xk = x(:,k);
        xkouter = xk*xk';
        fk = f(:,k);
        
        dWi = zeros(1,nBasis);
        for i = 1:nBasis
            dWi(i) = trace( Sinvk*model.basis(:,:,i) - model.basis(:,:,i)*xkouter )*w(i,k);
        end
        dW = dW + bsxfun(@times, dWi, fk);
        
    end
else
    error( 'Model type must be covariance or information' );
end

dW = dW*0.5/nData; % Deferred for fewer operations
dW = dW + model.w*model.wPenalty;