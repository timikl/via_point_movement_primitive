%https://it.mathworks.com/help/robotics/ug/choose-trajectories-for-manipulator-paths.html Franka waypoints 

wpts = [; 45 1 15 1];
tpts = 0:3;

numsamples = 100;
[q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(wpts,tpts,numsamples);
plot(q, tsamples)
hold on
plot(wpts,timepoints,'x')
xlabel('t')
ylabel('Positions')
legend('X-positions','Y-positions')
hold off