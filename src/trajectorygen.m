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

for 
t=traj(1)+
end