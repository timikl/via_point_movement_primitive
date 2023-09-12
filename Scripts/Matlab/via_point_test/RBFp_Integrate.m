function [S,psi] = RBFp_Integrate(RBF,S,nu)

% discrete move RBF realization, NOT weighted with the phase
if nargin < 3
%    nu = 0;
    nu = 1;
end
%% init params for target traj. and fitting
epsilon = 1.e-10;
NS = size(RBF.w,2); %number of signals

% the weighted sum of the locally weighted regression models
psi=exp(-(S.x-RBF.c).^2./(2*RBF.sigma2))';
for i = 1:NS
    S.y(i) = sum((RBF.w(:,i)).*psi); 
end
S.basis = psi*S.x;
% phase variable must be last updated
dx = -RBF.a_x*S.x/RBF.tau*nu;
S.x=S.x+dx*RBF.dt;


