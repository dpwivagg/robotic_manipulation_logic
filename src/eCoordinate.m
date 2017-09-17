% Function eCoordinate accepts two angles and the link lengths
% and returns the x, y, z positon of the tip as a 3x1 matrix
function e = eCoordinate(l1, l2, q0, q1)

% Calculate the value of x
eX0 = l2 * cosd(q1) * cosd(q0);

% Calculate the value of y
eY0 = l2 * cosd(q1) * sind(q0);

% Calculate the value of z
eZ0 = l1 + l2 * sind(q1);

% Store the x, y, and z values in a matrix
e = [eX0 eY0 eZ0];

end