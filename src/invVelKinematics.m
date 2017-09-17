% This function takes a 3x1 task space tip velocity vector and the three
% current joint angles and returns the three joint angular velocities
function invVelKinematics(tipV, q0, q1, q2)
l2 = 1; l3 = 1;
% Create a matrix of the velocity kinematics of the arm
Vq = [-sind(q0) * (l2*cosd(q1) + l3*cosd(q1 - q2)), -cosd(q0) * (l2*sind(q1) + l3*sind(q1 - q2)), cosd(q0) * l3*sind(q1 - q2);...
       cosd(q0) * (l2*cosd(q1) + l3*cosd(q1 - q2)), -sind(q0) * (l2*sind(q1) + l3*sind(q1 - q2)), sind(q0) * l3*sind(q1 - q2);...
                                                 0,               l2*cosd(q1) + l3*cosd(q1 - q2),           -l3*cosd(q1 - q2)]
det(Vq)
X = inv(Vq)
end