% This function generates a 4x4 homogenous transformation matrix for each
% joint and multiplies them together to create one 4x4 homogenous
% transformation matrix representing the forward position kinematics of the
% arm. This function takes in the angles for the base, shoulder, and elbow
% joints and returns a 4x4 transformation matrix
function TM = symForPosKinematics(mode)
    % Get global link lengths
    %linkVal = getLinkValues();
    %q = getJointValues();
    % Define the DH parameters for the arm 
    syms t1 t2 t3 l1 l2 l3 a1
    % For link 1
    link1.a = 0; link1.alpha = a1; link1.d = l1; link1.theta = t1; 
    % For link 2
    link2.a = l2; link2.alpha = 0;  link2.d = 0; link2.theta = t2;
    % For link 3
    link3.a = l3; link3.alpha = 0;  link3.d = 0; link3.theta = t3;

    % Generate a 4x4 matrix for each joint
    M1 = fourByFourForPos(link1.a, link1.alpha, link1.d, link1.theta);
    M2 = fourByFourForPos(link2.a, link2.alpha, link2.d, link2.theta);
    M3 = fourByFourForPos(link3.a, link3.alpha, link3.d, link3.theta);

    % Multiply the matrices together to create a final transformation matrix
    if (mode == 1)
        TM = M1 * M2 * M3;
    else
        TM = M1 * M2;
    end
end

% This function creates a 4x4 transformation matrix for a joint using the
% DH parameters for its link.
function M = fourByFourForPos(a, alpha, d, theta)

M = [cos(theta), (-sin(theta) * cos(alpha)), (sin(theta) * sin(alpha)), (a * cos(theta));...
     sin(theta), (cos(theta) * cos(alpha)), (-cos(theta) * sin(alpha)), (a * sin(theta));...
     0, sin(alpha), cos(alpha), d;...
     0, 0, 0, 1];

end