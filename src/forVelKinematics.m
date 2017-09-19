% This function takes three joint angular velocities and three joint 
% angles and returns a 3x1 velocity vector
function VM = forVelKinematics(q0, q1, q2, v0, v1, v2)
% Define link lengths
l1 = 1; l2 = 1; l3 = 1;

% Calculate the symbolic Jacobian matrix
syms symq0 symq1 symq2 syml1 syml2 syml3
J = jacobian([cos(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
              sin(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
              syml1 + (syml2 * sin(symq1)) + (syml3 * sin(symq1 - symq2))],[symq0 symq1 symq2]);

% Convert q0, q1, q2 to radians
q0 = q0 * (pi/180);
q1 = q1 * (pi/180);
q2 = q2 * (pi/180);

% Substitute real values into symbolic Jacobian
J = subs(J, [symq0 symq1 symq2 syml1 syml2 syml3], [q0 q1 q2 l1 l2 l3]);
 
% Create a vector of angular velocities
V = [v0; v1; v2];

% Multiply the vectors together to calculate the current velocity
VM = J * V;
end