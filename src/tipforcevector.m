%% Joint Torque to force at tip calculations
function f = tipforcevector(torque)
% get the global variables
linkVal = [20 17 20];%getLinkValues();
q = [0 90 -90];%getJointValues();
q(3) = q(3) + 90;
% recalcuate the jacobian

% Calculate the symbolic Jacobian matrix
syms symq0 symq1 symq2 syml1 syml2 syml3
J = jacobian([cos(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
              sin(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
              syml1 + (syml2 * sin(symq1)) + (syml3 * sin(symq1 - symq2))],[symq0 symq1 symq2]);

% get the transpose then the inverse
Jt = transpose(J);
Ji = inv(Jt);
          
% Convert q0, q1, q2 to radians
q0 = q(1) * (pi/180);
q1 = q(2) * (pi/180);
q2 = q(3) * (pi/180);

% Substitute real values into symbolic Jacobian
Jsub = subs(Ji, [symq0 symq1 symq2 syml1 syml2 syml3], [q0 q1 q2 linkVal(1)/100 linkVal(2)/100 linkVal(3)/100]);

% torque times inversed transpose of J is force at tip
f = double(Jsub*torque);
end
