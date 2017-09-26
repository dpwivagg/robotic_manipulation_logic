%% Trajectory plot x,y and Z
% plots trajectory 
function y = plottraj(postart, posend, tstart, tend, tstep, velstart, velend)
% ganther all 3 trajectories
xtraj = trajectorygen(tstart,tend,velstart, velend, postart(1), posend(1), tstep);
ytraj = trajectorygen(tstart,tend,velstart, velend, postart(2), posend(2), tstep);
ztraj = trajectorygen(tstart,tend,velstart, velend, postart(3), posend(3), tstep);
% now plot them 
y = [xtraj, ytraj, ztraj];
plot(y(:,1), y(:,4)*10,y(:,5), y(:,8)*10,'--', y(:,9), y(:,12)*10,'p:');
grid on;
end
