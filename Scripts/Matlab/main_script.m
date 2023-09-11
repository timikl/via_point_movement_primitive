% main_script.m

% Define the parameters
x0 = [0, 0, 0];
dx0 = [0, 0, 0];
ddx0 = [0, 0, 0];
x1 = [10, 10, 10];
dx1 = [0, 0, 0];
ddx1 = [0, 0, 0];
t = 0.5;
dt = 0.01;

% Create an instance of the Polynomial class
poly = Polynomial(x0, dx0, ddx0, x1, dx1, ddx1, t, dt);

% Call methods of the class
poly = poly.whole_trajectory_calculate();
poly = poly.velocity_calculate();
poly = poly.acceleration_calculate();
poly = poly.jerk_calculate();

% Plotting
figure;

% Trajectory plot
subplot(4,1,1);
plot(poly.time_split, poly.trajectory);
title('Trajectory');
xlabel('Time');
ylabel('Position');

% Velocity plot
subplot(4,1,2);
plot(poly.time_split, poly.velocity);
title('Velocity');
xlabel('Time');
ylabel('Velocity');

% Acceleration plot
subplot(4,1,3);
plot(poly.time_split, poly.acceleration);
title('Acceleration');
xlabel('Time');
ylabel('Acceleration');

% Jerk plot
subplot(4,1,4);
plot(poly.time_split, poly.jerk);
title('Jerk');
xlabel('Time');
ylabel('Jerk');

