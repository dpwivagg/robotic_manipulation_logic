% This function takes a point in 3D space and computes the joint angles 
% necessary at each link to place the
% tip of the robot arm at the point
function joints = invPosKinematics(point)
if(point(1) > 37 || point(1) < 0 || point(2) > 37 || point(2) < -37 || point(3) > 57 || point(3) < 0)
    error('Out of bounds!')
    clear java;
end

l1 = 20;
l2 = 17;
l3 = 20;

% Calculate the angle of the plane for the shoulder-elbow 2DOF arm,
% corresponding to the angle of the base link
q0 = atan2(point(2), point(1));

% Calculate the distance between the origin and the point on the xy plane
r = sqrt(point(1)^2 + point(2)^2);


% Calculate the angle of the second link 
q2 = acos(((r^2 + (l1 - point(3))^2)-(l2^2 + l3^2)) / (2 * l2 * l3));

% Calculate the angle between the x axis of link one and the line
% connecting the origin of link one to the point
B = atan2((l1 - point(3)), r);

% Calculate the angle between the first link and the line
% connecting the origin of link one to the point
gamma = acos(((l1 - point(3))^2 + r^2 + l2^2 - l3^2)/(2 * l2 * sqrt((l1 - point(3))^2 + r^2)));

% Calculate the angle of the first link
% if q2 < 0
    q1 = B - gamma;
% else 
%    q1 = B + gamma;
% end

q0 = -q0 * (180/pi);
q1 = -q1 * (180/pi);
q2 = (-q2 * (180/pi)) + 90;
if(isreal(q0) && isreal(q1) && isreal(q2))
    joints = [q0; q1; q2];
else
    error('That configuration is impossible.')
end
end