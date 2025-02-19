function a543210 = polynomial1(x0,dx0,ddx0,x1,dx1,ddx1,t)
% y = a5 * t^5 + a4 * t^4 + a3 * t^3 + a2 * t^2 + a1 * t + a0

a0 = x0;
a1 = dx0;
a2 = ddx0 / 2;
a3 = ( 20 * x1 - 20 * x0 - (8 * dx1 + 12 * dx0) * t - (3 * ddx0 - ddx1) * t^2)  / (2 * t^3);

a4 = ( 30 * x0 - 30 * x1 + (14 * dx1 + 16 * dx0) * t + (3 * ddx0 - 2 * ddx1) * t^2 ) / (2 * t^4);

a5 = ( 12 * x1 - 12 * x0 - (6 * dx1 + 6 * dx0) * t - (ddx0 - ddx1) * t^2 ) / (2 * t^5);

a543210 = {a5, a4, a3, a2, a1, a0};

