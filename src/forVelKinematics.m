% This function takes three joint angular velocities and three joint 
% angles and returns a 3x1 velocity vector
function VM = forVelKinematics(q0, q1, q2, v0, v1, v2)
l1 = 1; l2 = 1; l3 = 1;
% Create the Jacobian matrix for the arm
Vq = [-sind(q0) * (l2*cosd(q1) + l3*cosd(q1 - q2)), -cosd(q0) * (l2*sind(q1) + l3*sind(q1 - q2)), cosd(q0) * l3*sind(q1 - q2);...
       cosd(q0) * (l2*cosd(q1) + l3*cosd(q1 - q2)), -sind(q0) * (l2*sind(q1) + l3*sind(q1 - q2)), sind(q0) * l3*sind(q1 - q2);...
                                                 0,               l2*cosd(q1) + l3*cosd(q1 - q2),           -l3*cosd(q1 - q2)];
 
% Create a vector of angular velocities
Vv = [v0; v1; v2];

% Multiply the vectors together to calculate the current velocity
VM = Vq * Vv;
end