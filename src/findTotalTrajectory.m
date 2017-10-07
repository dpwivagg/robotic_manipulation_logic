% This function takes a matrix of desired setpoints and generates a
% trajectory that passes through all of them
% pointMatrix is a nx3 matrix with columns of x, y, and z setpoints
% TODO: Add a time argument that allows the user to specify the amount of
% time for the total trajectory. We will have to assume that each setpoint
% has the same desired time unless we expand to set them individually

function pointMatrix = findTotalTrajectory(points)
    % The starting position is assumed to be the first position
    lastA = points(1,:);
    pointMatrix = [];
    for k = 2:size(points, 1)
        A = points(k,:);
        x = trajectorygen(0,4,0,0,lastA(1),A(1),0.1);
        y = trajectorygen(0,4,0,0,lastA(2),A(2),0.1);
        z = trajectorygen(0,.4,0,10,lastA(3),A(3),0.01);
        lastA = A;
        pointMatrix = [pointMatrix; x(:,2) y(:,2), z(:,2)];
    end
end

%% Tragectory Generation Function
%   solves for a cubic polynomial trajectory. Takes in start and end times	
% start	and	end	velocities	(in	mm/sec), and start and	end positions as 
% well as a time step. outputs an nx4 array,row structure [time	(sec),	
% pos(t),vel(t), accel(t)],	n # of time points
%
function t = trajectorygen(tstart,tstop,vstart, vend, postart, posend, tstep)
% q(t) = a0+a1(t)+a2(t^2)+a3(t^3)
% velcoity = a1 +2a2(t)+3a3(t^2)
% general 4 by 4 matrix to find a0 a1 a2 and a3
matrix = [1, tstart, tstart^2, tstart^3; 0, 1, 2*tstart, 3*tstart^2;...
    1, tstop, tstop^2, tstop^3; 0, 1, 2*tstop, 3*tstop^2];
% 1 by 4 matrix to find a0 a1 a2 and a3
B = [postart; vstart;  posend; vend];
% get the inverse of the 4 by 4 matrix
A = inv(matrix);
 % calculate a0 a1 a2 and a3
traj = A*B;

% intalize time, position, velocity and acceleration
time = tstart;
pos = postart;
velo = vstart;
acc = 2*traj(3) + 6*traj(4)*tstart;
t = [tstart, pos, velo, acc];
 
% generate n by 4 matrix for trajectory
for a = tstart+tstep:tstep:tstop
tpos=traj(1)+traj(2)*a+traj(3)*a^2+traj(4)*a^3;
tvel = traj(2)+2*traj(3)*a+3*traj(4)*a^2;
tacc = 2*traj(3) +  6*traj(4)*a;
t = [t; a, tpos, tvel, tacc];
end
% end function
end