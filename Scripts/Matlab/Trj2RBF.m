
function [RBF,qRBF] = Trj2RBF(y,dt,NN)
%% Trajektorijo q realiziram z RBF-ji
if nargin < 3
    NN = 30;
end

joints = size(y,2);
%learning
RBF.N = NN;RBF.dt=dt;RBF.a_x=2;

[RBF]=RBF_Train(y,RBF);

Y = [];
S.x = 1;

%zaradi zaokrozitve dodam ?e en sample in imam zato neenacaj !!!!)
while S.x >= exp(-RBF.a_x*(1+RBF.dt/RBF.tau))
   [S]=RBF_Integrate(RBF,S);
   Y = [Y;S.y];
end  
% 
want_plot = 0;
if want_plot
    figure(99)
    plot(y,'k'); hold on;
    plot(Y,'r:');
    title('trajectory');
end

qRBF = [Y];

