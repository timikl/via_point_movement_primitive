function [x, v, a, t] = min_jerk_traj(x0, xf, v0, vf, a0, af, tf, dt)

% Compute constants
t0 = 0;
t1 = tf - t0;
t2 = t1 + t1;
t3 = t2 + t1;
t4 = t3 + t1;
t5 = t4 + t1;

A = [0 0 0 0 0 1;
     t1^5 t1^4 t1^3 t1^2 t1 1;
     0 0 0 0 1 0;
     5*t1^4 4*t1^3 3*t1^2 2*t1 1 0;
     0 0 0 2 0 0;
     20*t1^3 12*t1^2 6*t1 2 0 0];

b = [x0 xf v0 vf a0 af]';

c = A\b;

% Compute trajectory
t = t0:dt:tf;
x = c(1)*t.^5 + c(2)*t.^4 + c(3)*t.^3 + c(4)*t.^2 + c(5)*t + c(6);
v = 5*c(1)*t.^4 + 4*c(2)*t.^3 + 3*c(3)*t.^2 + 2*c(4)*t + c(5);
a = 20*c(1)*t.^3 + 12*c(2)*t.^2 + 6*c(3)*t + 2*c(4);

end 
