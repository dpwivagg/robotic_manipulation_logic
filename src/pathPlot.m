% Function pathPlot takes in x, y, and z positions of the arm elbow and tip
% from a .csv file and creates a 3D plot of the path
function f = pathPlot(xEpos, yEpos, zEpos, xTpos, yTpos, zTpos, xPpos, yPpos, zPpos)
% Create circles showing range of arm
theta = 0:pi/50:2*pi;

% For the xy plane starting at the shoulder
xyplane.x = 2 * cos(theta);
xyplane.y = 2 * sin(theta);
xyplane.z = ones(1,numel(xyplane.x));

% For the xz plane starting at the shoulder
xzplane.x = 2 * cos(theta);
xzplane.y = zeros(1,numel(xzplane.x));
xzplane.z = 2 * sin(theta) + 1;

% For the yz plane starting at the shoulder
yzplane.x = zeros(1,numel(xzplane.x));
yzplane.y = 2 * cos(theta);
yzplane.z = 2 * sin(theta) + 1;

% Create a plot for the position of the link over time
clf;
f = plot3(xEpos, yEpos, zEpos,...
          xTpos, yTpos, zTpos,...
          xPpos, yPpos, zPpos,...
          xyplane.x, xyplane.y, xyplane.z, 'k:',...
          xzplane.x, xzplane.y, xzplane.z, 'k:',...
          yzplane.x, yzplane.y, yzplane.z, 'k:');
hold on;                   % Hold on to objects in the axes
box on;                    % Put a box around axes
grid on;                   % Put gridlines on the figure
axis([-2 2 -2 2 0 3]);     % Set axes limits
xlabel('X'); ylabel('Y'); zlabel('Z'); % Add labels to the axes
title({'Angle of link over time'}); % Add title to the figure
view(37.5, 30);            % Rotate the plot to 'face forward'
end