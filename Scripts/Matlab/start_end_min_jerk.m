% Define the start node
start_point = [0, 15];
%Define the end node
end_point = [45, 0];

t = linspace(start_point(1), end_point(1), 100);


plot(start_point(1), start_point(2), 'o', 'MarkerFaceColor', 'red');
%end_y = linspace(0, 30, 3);

hold on
plot(end_point(1), end_point(2), 'o', 'MarkerFaceColor', 'blue');

v0 = [0, 0];
vf = [0, 0];
a0 = [0, 0];
af = [0, 0];
tf = 5;  % Duration of trajectory in seconds
dt = 0.01;

[x, v, a, t] = min_jerk_traj(start_point, end_point, v0, vf, a0, af, tf, dt);

wpts = [0 45; 15 0];
[q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(wpts, start_point(1):end_point(1), 100);