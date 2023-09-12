x0 = [0, 0];
dx0 = [0, 0]; 
ddx0 = [0, 0];
x1 = [10, 10];
dx1 = [0, 0];
ddx1 = [0, 0];
t = 3;
steps = 100;
x = zeros(100);
y= zeros(100);

a = polynomial1(x0,dx0,ddx0,x1,dx1,ddx1,t);
x,y = evaluate(a,t,steps);


plot(y)