% This function takes a point in 3D space and the three link lengths of the
% robot and computes the joint angles necessary at each link to place the
% tip of the robot arm at the point
function joints = invPosKinematics(point, l1, l2, l3)

% Calculate the angle of the plane for the shoulder-elbow 2DOF arm,
% corresponding to the angle of the base link
q0 = atan2(point.y, point.x);

% Calculate the distance between the origin and the point on the xy plane
r = sqrt(point.x^2 + point.y^2);

% Calculate the angle of the second link 
q2 = acos(((r^2 + (l1 - point.z)^2)-(l2^2 + l3^2)) / (2 * l2 * l3));

% Calculate the angle between the x axis of link one and the line
% connecting the origin of link one to the point
B = atan2((l1 - point.z), r);

% Calculate the angle between the first link and the line
% connecting the origin of link one to the point
gamma = acos(((l1 - point.z)^2 + r^2 + l2^2 - l3^2)/(2 * l1 * sqrt(point.z^2 + r^2)));

% Calculate the angle of the first link
if q2 < 0
    q1 = B - gamma;
else 
    q1 = B + gamma;
end

q0 = q0 * (pi/180);
q1 = q1 * (pi/180);
q2 = q2 * (pi/180);

joints = [q0; q1; q2];
end