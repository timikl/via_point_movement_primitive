% Define the start node
start_x = 0;
start_y = 15;
scatter(start_x, start_y, 'o', 'Color','red')

% Define the range of x and y coordinates
x_range = linspace(30, 60, 10);
y_range = linspace(0, 30, 10);

% Create a grid of points
[X, Y] = meshgrid(x_range, y_range);

% Plot the grid points
hold on
scatter(X(:), Y(:));