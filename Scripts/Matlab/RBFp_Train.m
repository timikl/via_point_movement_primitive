function [RBF]=RBFp_Train(y, RBF, nu)

% Global regression nonormalized RBF (PMP) 
% input measured values 
%   y signal
%   RBF, ... RBF parameters 

% RBF parameters
%   N,  ... number of Gaussian kernel functions
%   w,  ... weight vector of size(Nx1)
%   c,
%   sigma2
%   tau

if nargin < 3
    nu = ones(size(y,1),1);
end
%% params - global
[NT,NS] = size(y);

RBF.tau = (NT-1)*RBF.dt;
RBF.w = zeros(RBF.N,NS);     % initial weights

%%% gausian kernel functions
c_lin=linspace(0,1,RBF.N);
RBF.c=exp(-RBF.a_x * c_lin);
RBF.sigma2=(diff(RBF.c)*0.75).^2;
RBF.sigma2=[RBF.sigma2,RBF.sigma2(end)];

%% init params for target traj. and fitting
x = 1;
%% fit all points of the trajectory
for t=1:NT
    %% the weighted sum of the locally weighted regression models
    psi=exp(-0.5*(x-RBF.c).^2./RBF.sigma2)';
    xx = psi/sum(psi);
    %A(t,:) = xx;
    A(t,:) = psi;
    %% update phase at last !!!!
    dx = -RBF.a_x*x/RBF.tau*nu(t);
    x = x + dx*RBF.dt;
end  
AI =  pinv(A);
RBF.w = AI*y;

    

