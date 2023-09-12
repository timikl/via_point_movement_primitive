addpath('../_nDMP')
%trajektorija
dt = 0.01;
tf = 2;
t = 0:dt:tf;
j = 0;

%Plots 10 sinuses
for a = 1:0.1:2
    j = j + 1;
    for i = 1:size(t)
        p(j,:) = a*sin(t*2*pi/tf);
    end
end
plot(t,p);


mu = mean(p);
sigma = std(p);

hold on
plot(t,mu,'LineWidth',2);

%rbf
W = [];
M = 10;
for j = 1:size(p,1)
    [RBF,~] = Trj2RBF(p(j,:)',dt,M);
    W(:,j) = RBF.w;
end

%RBF11 = RBF;
%p11 = RBF2Trj(RBF11)

mu_w = mean(W')'; %NAROBE ????
%sigma_w = std(W');  %NAROBE ????

%mu_w = 1/M*sum(W,2); %Zdaj je ta NAROBE !!!
sigma_w = zeros(M,M);
for i = 1:M
    sigma_w = sigma_w + 1/M*(W(i,:) - mu_w)*(W(i,:) - mu_w)';
end

RBF_mu = RBF; RBF_mu.w = mu_w;

p_w = RBF2Trj(RBF_mu);

plot(t(1:size(p_w)), p_w,':k','LineWidth',3);


%predikcija mu_x glede na viapoint
%x_via = 1.4;
%y_via = 2;
x_via = 1.5;
y_via = -1.5;
%Za to rabim x in y, x je faza, odvisna od casa (x-os)
S.x = exp(-RBF.a_x*x_via/RBF.tau);
%sigma_y je sigma(x_via) manjši sigma, bolj sili v via tocko - to je design
%parameter in ga ni treba racunati
%sigma_y = sigma(x_via/dt);
sigma_y = 0.001;

psi=exp(-(S.x-RBF_mu.c).^2./(2*RBF_mu.sigma2))';
L = sigma_w*psi/(sigma_y + psi'*sigma_w'*psi);
mu_v = mu_w + L*(y_via - psi'*mu_w);


%test
RBF_via = RBF_mu; RBF_via.w = mu_v;

p_via = RBF2Trj(RBF_via);

plot(t(1:size(p_via)), p_via,':m','LineWidth',2);

