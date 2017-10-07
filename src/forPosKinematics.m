% This function generates a 4x4 homogenous transformation matrix for each
% joint and multiplies them together to create one 4x4 homogenous
% transformation matrix representing the forward position kinematics of the
% arm. This function takes in the angles for the base, shoulder, and elbow
% joints and returns a 4x4 transformation matrix
function TM = forPosKinematics(mode)
% Get global link lengths
linkVal = getLinkValues();
q = getJointValues();
% Define the DH parameters for the arm 
% For link 1
l1.a = 0; l1.alpha = 90; l1.d = linkVal(1); l1.theta = q(1); 
% For link 2
l2.a = linkVal(2); l2.alpha = 0;  l2.d = 0; l2.theta = q(2);
% For link 3
l3.a = linkVal(3); l3.alpha = 0;  l3.d = 0; l3.theta = -(q(3)+90);
% For tool tip
tt.a = 0; tt.alpha = 0;  tt.d = 0; tt.theta = 90;

% Generate a 4x4 matrix for each joint
M1 = fourByFourForPos(l1.a, l1.alpha, l1.d, l1.theta);
M2 = fourByFourForPos(l2.a, l2.alpha, l2.d, l2.theta);
M3 = fourByFourForPos(l3.a, l3.alpha, l3.d, l3.theta);
M4 = fourByFourForPos(tt.a, tt.alpha, tt.d, tt.theta);

% Multiply the matrices together to create a final transformation matrix
if (mode == 1)
TM = M1 * M2 * M3 * M4;
else
TM = M1 * M2;
end
end

% This function creates a 4x4 transformation matrix for a joint using the
% DH parameters for its link.
function M = fourByFourForPos(a, alpha, d, theta)

M = [cosd(theta), (-sind(theta) * cosd(alpha)), (sind(theta) * sind(alpha)), (a * cosd(theta));...
     sind(theta), (cosd(theta) * cosd(alpha)), (-cosd(theta) * sind(alpha)), (a * sind(theta));...
     0, sind(alpha), cosd(alpha), d;...
     0, 0, 0, 1];

end