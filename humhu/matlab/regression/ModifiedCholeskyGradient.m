function [dV, dW] = ModifiedCholeskyGradient( x, f, C, model )
% Calculates the parameter element-wise gradient of the modified Cholesky
% generator
% x - Gaussian samples
% f - predictor features [unscaled!]
% C - Gaussian parameter offsets
% modelParams - struct of model parameters
% modelMode - 'covariance' or 'information'
% dataMode - 'error' or 'innovation'

nx = size(x,1); % Dimensionality of Gaussian
nd = size(x,2); % Number of data points
nv = size(model.v, 2); % Number of L weights
nw = nx; % Number of D weights
nf = size(f,1); % Dimensionality of features

[R, L, D, lInds] = ModifiedCholeskyRegression( f, model );
S = R + C;

% Scale features after regression for gradient calculations
f = bsxfun( @rdivide, f, model.featureScales );

dV = zeros(nf,nv);
dW = zeros(nf,nw);

if strcmp( model.modelMode, 'covariance' )
    
    % TODO Vectorize
    for k = 1:nd
        Lk = L(:,:,k);
        Dk = D(:,:,k)';
        fk = f(:,k);
        
        xk = x(:,k);
        Ci = S(:,:,k);
        %         Cinv = inv(Ci);
        
        %         Cdiff = Cinv - Cinv*(xk*xk')*Cinv;
        %         Cdiff = Cinv*(eye(nx) - (xk*xk')*Cinv);
        Cdiff = Ci\( eye(nx) - (xk*xk')/Ci );
        
        ind = zeros(nx);
        for j = 1:nw
            %             ind(j,j) = 1;
            %             dR = Lk*ind*Lk'*Dk(j,j);
            %             ind(j,j) = 0;
            
            dR = L(:,j)*L(:,j)'*Dk(j,j);
            
            %             dW(:,j) = dW(:,j) - 0.5*trace( Cdiff*dR )*fk;
            dW(:,j) = dW(:,j) - 0.5*sum( sum( Cdiff.*dR' ) )*fk;
        end
        
        for j = 1:nv
            ind = zeros(nx);
            ind( lInds(j) ) = 1;
            dR = Lk*Dk*ind + ind'*Dk*Lk';
            
            %             dV(:,j) = dV(:,j) - 0.5*trace( Cdiff*dR )*fk;
            dV(:,j) = dV(:,j) - 0.5*sum( sum( Cdiff.*dR' ) )*fk;
        end
        
    end
    
    dW = dW/nd;
    dV = dV/nd;
    
elseif strcmp( model.modelMode, 'information' )
    
    dInds = 1:nx+1:nx*nx;
    dInc = nx*nx;
    
    [lI, lJ] = ind2sub( [nx,nx], lInds );
    
    % TODO Vectorize and update to use LDLT
    for k = 1:nd
        Lk = L(:,:,k);
        Dk = D(dInds)';
        dInds = dInds + dInc;
        xk = x(:,k);
        fk = f(:,k);
        
        epsilon = Lk*xk;
        epsilon_sq = epsilon.^2;
        
        wScale = 0.5*(1 - epsilon_sq.*Dk)';
        dW = dW + bsxfun(@times, wScale, fk);
        
        vScale = -(epsilon(lI).*Dk(lI).*xk(lJ))';
        dV = dV + bsxfun(@times, vScale, fk);
        
    end
    
    dW = dW/nd;
    dV = dV/nd;
    
else
    error( 'Model mode must be either covariance or information' );
end

dV = dV - model.vPenalty*model.v;
dW = dW - model.wPenalty*model.w;