% This function takes a 3x1 task space tip velocity vector and the three
% current joint angles and returns the three joint angular velocities
function iVM = invVelKinematics(tipV, q0, q1, q2)
% Define link lengths
l1 = 1; l2 = 1; l3 = 1;

% Calculate the symbolic Jacobian
syms symq0 symq1 symq2 syml1 syml2 syml3
J = jacobian([cos(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
              sin(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
              syml1 + (syml2 * sin(symq1)) + (syml3 * sin(symq1 - symq2))],[symq0 symq1 symq2]);

% Calculate the symbolic inverse of the Jacobian
iJ = inv(J);

% Convert q0, q1, q2 to radians
q0 = q0 * (pi/180);
q1 = q1 * (pi/180);
q2 = q2 * (pi/180);

% Substitute values into symbolic Jacobian
iJ = subs(iJ, [symq0 symq1 symq2 syml1 syml2 syml3], [q0 q1 q2 l1 l2 l3]);

% Multiply the inverse Jacobian by the tip velocity vector
iVM = iJ * tipV;
end