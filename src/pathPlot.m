% Function pathPlot takes in x, y, and z positions of the arm elbow and tip
% from a .csv file and creates a 3D plot of the path
function f = pathPlot(xTpos, yTpos, zTpos)
% Get global link lengths
linkVal = getLinkValues();

% Create circles showing range of arm
theta = 0:pi/50:2*pi;

% For the xy plane starting at the shoulder
xyplane.x = (linkVal(1) + linkVal(2)) * cos(theta);
xyplane.y = (linkVal(1) + linkVal(2)) * sin(theta);
xyplane.z = ones(1,numel(xyplane.x)) * linkVal(3);

% For the xz plane starting at the shoulder
xzplane.x = (linkVal(1) + linkVal(2)) * cos(theta);
xzplane.y = zeros(1,numel(xzplane.x));
xzplane.z = (linkVal(1) + linkVal(2)) * sin(theta) + linkVal(3);

% For the yz plane starting at the shoulder
yzplane.x = zeros(1,numel(xzplane.x));
yzplane.y = (linkVal(1) + linkVal(2)) * cos(theta);
yzplane.z = (linkVal(1) + linkVal(2)) * sin(theta) + linkVal(3);

% Create a plot for the position of the link over time
clf;
f = plot3(xTpos, yTpos, zTpos,...
          xyplane.x, xyplane.y, xyplane.z, 'k:',...
          xzplane.x, xzplane.y, xzplane.z, 'k:',...
          yzplane.x, yzplane.y, yzplane.z, 'k:');
hold on;                   % Hold on to objects in the axes
box on;                    % Put a box around axes
grid on;                   % Put gridlines on the figure
axis([-40 40 -40 40 0 60]);     % Set axes limits
xlabel('X'); ylabel('Y'); zlabel('Z'); % Add labels to the axes
title({'Angle of link over time'}); % Add title to the figure
view(37.5, 30);            % Rotate the plot to 'face forward'
end