% This function creates a 4x4 transformation matrix for a joint using the
% DH parameters for its link.
function M = fourByFourForPos(a, alpha, d, theta)

M = [cosd(theta), (-sind(theta) * cosd(alpha)), (sind(theta) * sind(alpha)), (a * cosd(theta));...
     sind(theta), (cosd(theta) * cosd(alpha)), (-cosd(theta) * sind(alpha)), (a * sind(theta));...
     0, sind(alpha), cosd(alpha), d;...
     0, 0, 0, 1];

end