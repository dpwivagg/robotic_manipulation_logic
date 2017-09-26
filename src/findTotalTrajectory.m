% This function takes a matrix of desired setpoints and generates a
% trajectory that passes through all of them
% pointMatrix is a nx3 matrix with columns of x, y, and z setpoints
% TODO: Add a time argument that allows the user to specify the amount of
% time for the total trajectory. We will have to assume that each setpoint
% has the same desired time unless we expand to set them individually

function pointMatrix = findTotalTrajectory(points)
    % The starting position is assumed to be the first position
    lastA = points(1,:);
    pointMatrix = [];
    for k = 2:size(points, 1)
        A = points(k,:);
        x = trajectorygen(0,2,0,0,lastA(1),A(1),0.1);
        y = trajectorygen(0,2,0,0,lastA(2),A(2),0.1);
        z = trajectorygen(0,2,0,0,lastA(3),A(3),0.1);
        lastA = A;
        pointMatrix = [pointMatrix; x(:,2) y(:,2), z(:,2)];
    end
end