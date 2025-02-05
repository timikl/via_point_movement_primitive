% Define time steps
t = [0, 1, 2, 3]; % Adjust as needed

% Define initial and final points
x0 = [0, 0]; % Initial position [x, y]
xf = [10, 10]; % Final position [x, y]

% Define initial and final velocities and accelerations
v0 = [0, 0]; % Initial velocity [vx, vy]
vf = [0, 0]; % Final velocity [vx, vy]

% Generate a minimum jerk trajectory using cubic spline interpolation
t_interp = linspace(t(1), t(end), 100); % Interpolation time steps
x_interp = zeros(length(t_interp), 2); % Initialize interpolated positions

for dim = 1:2 % Loop through x and y dimensions
    p = [x0(dim), v0(dim), 0.5 * vf(dim), xf(dim)]; % Coefficients for cubic spline
    x_interp(:, dim) = polyval(p, t_interp); % Interpolate positions
end

% Plot the minimum jerk trajectory
figure;
plot(x_interp(:, 1), x_interp(:, 2), 'b-', 'LineWidth', 2);
hold on;
plot(x0(1), x0(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Initial point
plot(xf(1), xf(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Final point
xlabel('X');
ylabel('Y');
title('Minimum Jerk Trajectory Between Two Points');
legend('Minimum Jerk Trajectory', 'Start', 'End');
grid on;
axis equal;
hold off;