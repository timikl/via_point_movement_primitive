% Define the start node
start_point = [0, 15];
%Define the end node
end_point = [45, 0];

t = linspace(start_point(1), end_point(1), 100);


plot(start_point(1), start_point(2), 'o', 'MarkerFaceColor', 'red');



%end_y = linspace(0, 30, 3);


hold on
plot(end_point(1), end_point(2), 'o', 'MarkerFaceColor', 'blue');


path = interp1([start_point(1), end_point(1)], [start_point(2), end_point(2)], t, "linear");
plot(t, path)

%% Min Jerk Tray

% Define waypoints
waypoints = [start_point; end_point];

% Define time points
timePoints = [0 norm(end_point-start_point)];

% Define number of samples
numSamples = length(t);

% Compute minimum jerk trajectories
[q,qd,qdd,qddd,pp,tPoints,tSamples] = minjerkpolytraj(waypoints,timePoints,numSamples);

% Plot the trajectories for the x- and y-positions
plot(tPoints,q(:,1),'r',tPoints,q(:,2),'b');
hold off