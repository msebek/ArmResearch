function [ll,grad]=mlhgpobjective(params)

global hetGP;

ll = mlhgplikelihood(params',hetGP);
if nargout > 1,
    grad = mlhgpgradient(params',hetGP);
    grad = grad';    
end
