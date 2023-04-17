%addpath('c:\Users\Bojan Desktop\Dropbox\Documents\Work\_nDMP\');
dt = 0.01;
t = -1:dt:1;
y = sin(t*2*pi).*cos(6*t*2*pi);
plot(y,'r');

[RBF,yRBF] = Trj2RBF(y',dt,50);

hold on
plot(yRBF,'b:');
hold off
