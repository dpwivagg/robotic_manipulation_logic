% This function takes three joint angular velocities and three joint 
% angles and returns a 3x1 velocity vector
function VM = forVelKinematics(v0, v1, v2)
% Get global link lengths
linkVal = [.20;.17;.20];%getLinkValues();
q = [0;0.01;-.1];%getJointValues();
q(3) = -(q(3) + 90);
% Calculate the symbolic Jacobian matrix
syms symq0 symq1 symq2 syml1 syml2 syml3
J = jacobian([cos(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
              sin(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
              syml1 + (syml2 * sin(symq1)) + (syml3 * sin(symq1 - symq2))],[symq0 symq1 symq2]);

% Convert q0, q1, q2 to radians
q0 = q(1) * (pi/180);
q1 = q(2) * (pi/180);
q2 = q(3) * (pi/180);

% Substitute real values into symbolic Jacobian
J = subs(J, [symq0 symq1 symq2 syml1 syml2 syml3], [q0 q1 q2 linkVal(1) linkVal(2) linkVal(3)]);
 
% Create a vector of angular velocities
%V = [v0; v1; v2];

% Multiply the vectors together to calculate the current velocity
VM = J; %* V;
end