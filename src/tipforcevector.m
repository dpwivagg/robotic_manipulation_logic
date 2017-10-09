%% Joint Torque to force at tip calculations
function f = tipforcevector(torque)
% get the global variables
linkVal = [20 17 20];%getLinkValues();
linkVal = linkVal / 100;
q = [0 90 0];%getJointValues();
q(3) = -(q(3) + 90);
% recalcuate the jacobian

% Calculate the symbolic Jacobian matrix
% syms symq0 symq1 symq2 syml1 syml2 syml3
% J = jacobian([cos(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
%               sin(symq0) * (syml2 * cos(symq1) + syml3 * cos(symq1 - symq2));...
%               syml1 + (syml2 * sin(symq1)) + (syml3 * sin(symq1 - symq2))],[symq0 symq1 symq2]);
syms t1 t2 t3 l1 l2 l3 a1
TM = symForPosKinematics(1);
J = [diff(TM(1,4),t1), diff(TM(1,4),t2), diff(TM(1,4),t3);...
     diff(TM(2,4),t1), diff(TM(2,4),t2), diff(TM(2,4),t3);...
     diff(TM(3,4),t1), diff(TM(3,4),t2), diff(TM(3,4),t3)];

% get the transpose then the inverse
Jt = transpose(J);
Ji = inv(Jt);
          
% Convert q0, q1, q2 to radians
q0 = q(1) * (pi/180);
q1 = q(2) * (pi/180);
q2 = q(3) * (pi/180);

% Substitute real values into symbolic Jacobian
Jsub = subs(Ji, [t1 t2 t3 l1 l2 l3 a1], [q0 q1 q2 linkVal(1) linkVal(2) linkVal(3) pi/2]);

% torque times inversed transpose of J is force at tip
f = double(Jsub*torque);
end
