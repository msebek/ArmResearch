function [modelParams] = GaussianLikelihoodAscent( x, f, S, modelParams, ascentParams )
% Deprecated gradient ascent code for modified Cholesky

if ascentParams.showPlots
    if isfield(ascentParams, 'likelihoodAxes')
        likelihoodAxes = ascentParams.likelihoodAxes;
    else
        likelihoodFigure = figure;
        likelihoodAxes = axes( 'parent', likelihoodFigure );
    end
    hold( likelihoodAxes, 'on' );
    
    if isfield(ascentParams, 'paramsAxes')
        paramsAxes = ascentParams.paramsAxes;
    else
        paramsFigure = figure;
        paramsAxes = axes( 'parent', paramsFigure );
    end
end

% Calculate prior likelihood
[likelihood, penalized] = ObjectiveFunction( x, f, S, modelParams );
likelihood = mean( likelihood );
penalized = mean( penalized );
likelihoods = likelihood;
penalizeds = penalized;
prevPenalized = penalized;

if ascentParams.showPlots
    ls = plot( likelihoodAxes, 1, likelihood, 'b.-' );
    ps = plot( likelihoodAxes, 1, penalized, 'r.-' );
    xlabel( likelihoodAxes, 'Iteration' );
    ylabel( likelihoodAxes, 'Objective' );
    title( likelihoodAxes, [modelParams.dataMode, ' ', modelParams.modelMode, ' Gradient Ascent'] );
    legend( likelihoodAxes, 'Likelihood', 'Penalized', 'Location', 'Southeast' );
end

numIters = 1;

paramsVecs = [];

while true
    
    if numIters == 17
        blah = true;
    end
    
    f0 = penalized;
    
    [dV, dW] = GaussianLikelihoodGradient( x, f, S, modelParams );
    gradVec = [dV(:);dW(:)];
    gradNormSq = gradVec'*gradVec;
    
    t = ascentParams.stepSize;
    backtrackIters = 0;
    while true
        
        testParams = modelParams;
        
        vStep = t*dV;
        wStep = t*dW;
        
        testParams.v = testParams.v + vStep;
        testParams.w = testParams.w + wStep;
        
        [~, ftest] = ObjectiveFunction( x, f, S, testParams );
        ftest = mean(ftest);
        
        proj = f0 + ascentParams.backtrackAlpha*t*gradNormSq;
        if ftest > proj
            break
        end
        
        t = t*ascentParams.backtrackBeta;
        backtrackIters = backtrackIters + 1;
        if backtrackIters > ascentParams.maximumBacktracks
            warning( 'Hit max number of backtracks!' );
            break
        end
        
    end
    modelParams = testParams;
    
    % Calculate new likelihood
    [likelihood, penalized] = ObjectiveFunction( x, f, S, modelParams );
    likelihood = mean( likelihood );
    penalized = mean( penalized );
    likelihoods = [likelihoods, likelihood];
    penalizeds = [penalizeds, penalized];
    
    improvement = (penalized - prevPenalized);
    prevPenalized = penalized;
    
    numIters = numIters + 1;
    
    if ascentParams.showPlots
        set( ls, 'xdata', 1:numIters, 'ydata', likelihoods );
        set( ps, 'xdata', 1:numIters, 'ydata', penalizeds );
        drawnow;
        
        paramsVec = [modelParams.v(:); modelParams.w(:)];
        paramsVecs = [paramsVecs, paramsVec];
        plot( paramsAxes, paramsVecs' );
    end
    
    gradVec = [dV(:);dW(:)];
    gradNorm = norm( gradVec )/numel(gradVec);
    
    if gradNorm < ascentParams.gradientTolerance
        break
    end
    
    if numIters > ascentParams.maximumIterations
        break
    end
    
end

end

function [likelihood,penalized] = ObjectiveFunction(x, f, C, params)

if strcmp( params.dataMode, 'error' )
    S = ModifiedCholeskyRegression( f, params, true );
elseif strcmp( params.dataMode, 'innovation' )
    S = ModifiedCholeskyRegression( f, params, true ) + C;
end

likelihood = GaussianLogLikelihood( x, S, params.modelMode );

penalized = likelihood - params.vPenalty*0.5*params.v(:)'*params.v(:) ...
    - params.wPenalty*0.5*params.w(:)'*params.w(:);

end