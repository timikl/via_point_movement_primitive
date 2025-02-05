% Generiranje trajektorije
clear all;
dt = 0.01;
tf = 2;
t = 0:dt:tf;
j = 0;

%plots 11 sinuses
for a = 1:0.1:2
    j = j + 1;
    for i = 1:size(t)
        p(j,:) = a*sin(t*2*pi/tf); %stored sinuses
    end
end
plot(t,p);
legend("sin1", "sin1", "sin3","sin4", "sin5", "sin6","sin7", "sin8", "sin9", "sin10", "sin11")


% Mean and Standard deviation of sinuses
mu = mean(p);
sigma = std(p);

hold on
plot(t,mu,'LineWidth',2);
legend("mu")

% RBF
W = [];
M = 10;
for j = 1:size(p,1)
    [RBF,~] = Trj2RBFp(p(j,:)',dt,M);
    W(:,j) = RBF.w;
end

mu_w = mean(W')';
sigma_w = zeros(M,M);

%% Equation (5)
for i = 1:M
    sigma_w = sigma_w + 1/M*(W(i,:) - mu_w)*(W(i,:) - mu_w)';
end

RBF_mu = RBF; RBF_mu.w = mu_w;

p_w = RBFp2Trj(RBF_mu);

plot(t(1:size(p_w)), p_w,':k','LineWidth',3);
legend("RBF_mu")


%% predikcija mu_x glede na viapoint
%x_via = 1.4;
%y_via = 2;
x_via = 1.5;
y_via = -1.9;

% Za to rabim x in y, x je faza, odvisna od casa (x-os)
S.x = exp(-RBF.a_x*x_via/RBF.tau);
% sigma_y je sigma(x_via) manjsi sigma, bolj sili v via tocko; ni treba racunati

% sigma_y = sigma(x_via/dt);
sigma_y = 10.001;
% sigma_y = 10.001; %vecji sigma_y, manj natancno gre cez via tocko

%% Equations (6) from Learning via Mov primitives article
% psi = RBF(x,c,h) = exp(-((x - c)**2) / (2 * h**2))
% x is the input value.
% c is the center of the RBF.
% h is the width (or bandwidth) of the RBF.

psi=exp(-(S.x-RBF_mu.c).^2./(2*RBF_mu.sigma2))';
L = sigma_w*psi/(sigma_y + psi'*sigma_w'*psi);
mu_v = mu_w + L*(y_via - psi'*mu_w);


%% test
RBF_via = RBF_mu; RBF_via.w = mu_v;

p_via = RBFp2Trj(RBF_via);

plot(t(1:size(p_via)), p_via,':m','LineWidth',2);
legend("RBF_via")

% Set the legend
labels = cell(1, 14); 
for i = 1:11
    labels{i} = ['sin', num2str(i)];
end
labels{12} = 'mu';
labels{13} = 'RBF_mu';
labels{14} = 'RBF_via';

legend(labels);

hold off;
