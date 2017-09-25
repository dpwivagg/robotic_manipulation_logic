% Function pCoordinate accepts three angles and the link lengths
% and returns the x, y, z positon of the tip as a 3x1 matrix
function p = pCoordinate(q0, q1, q2)
% Get global link lengths
linkVal = getLinkValues();

% Calculate the value of x
pX0 = cosd(q0) * (linkVal(2) * cosd(q1) + linkVal(3) * cosd(q1 - q2));

% Calculate the value of y
pY0 = sind(q0) * (linkVal(2) * cosd(q1) + linkVal(3) * cosd(q1 - q2));

% Calculate the value of z
pZ0 = linkVal(1) + (linkVal(2) * sind(q1)) + (linkVal(3) * sind(q1 - q2));

% Store the x, y, z values in a matrix
p = [pX0 pY0 pZ0];

end