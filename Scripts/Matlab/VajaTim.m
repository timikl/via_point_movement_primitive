%addpath('c:\Users\Bojan Desktop\Dropbox\Documents\Work\_nDMP\');
dt = 0.01;
t = 0:dt:1;

x0 = 0;
dx0 = 0;
ddx0 = 0;
x1 = 10;
dx1 = 0;
ddx1 = 0;

a0 = x0;
a1 = dx0;
a2 = ddx0 / 2;
a3 = ( 20 * x1 - 20 * x0 - (8 * dx1 + 12 * dx0) * t - (3 * ddx0 - ddx1) * t.^2)  / (2 * t.^3);
a4 = ( 30 * x0 - 30 * x1 + (14 * dx1 + 16 * dx0) * t + (3 * ddx0 - 2 * ddx1) * t.^2 ) / (2 * t.^4);
a5 = ( 12 * x1 - 12 * x0 - (6 * dx1 + 6 * dx0) * t - (ddx0 - ddx1) * t.^2 ) / (2 * t.^5);

y = a5 * t  .^ 5 + a4 * t .^ 4 + a3 * t .^ 3 + a2 * t .^ 2 + a1 * t + a0;
plot(y, 'r');addpath('c:\Users\Bojan Desktop\Dropbox\Documents\Work\_nDMP\');
dt = 0.01;
t = 0:dt:1;
y = sin(t*2*pi).*cos(6*t*2*pi);
plot(y,'r');

[RBF,yRBF] = Trj2RBF(y',dt,20);

hold on
plot(yRBF,'b:');
hold off

[RBF,yRBF] = Trj2RBF(y',dt,5);

hold on
plot(yRBF,'b:');
hold off
