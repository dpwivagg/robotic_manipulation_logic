% Function threeLinkPlot takes the lengths of link 1 and 2,
% and the position of the elbow and tool tip to calculate the position
% of a 3-DOF arm with link lengths defined below.

% This function require function pCoordinate.m
function threeLinkPlot(l1, l2, v2, v3)

% Origin
v0 = [0 0 0];

% First link
v1 = [0 0 1];
x1 = [v0(1) v1(1)];
y1 = [v0(2) v1(2)];
z1 = [v0(3) v1(3)];

% Second link
x2 = [v1(1) v2(1)];
y2 = [v1(2) v2(2)];
z2 = [v1(3) v2(3)];

% Third link
x3 = [v2(1) v3(1)];
y3 = [v2(2) v3(2)];
z3 = [v2(3) v3(3)];

% Shadow link
xS = [v0(1) v3(1)];
yS = [v0(2) v3(2)];
zS = [v0(3) v0(3)];

% Create a circle showing the range of the arm
theta = 0:pi/50:2*pi;

% For the xy plane starting at the shoulder
xyplane.x = (l1 + l2) * cos(theta) + v1(1);
xyplane.y = (l1 + l2) * sin(theta) + v1(2);
xyplane.z = ones(1,numel(xyplane.x));

% For the xz plane starting at the shoulder
xzplane.x = (l1 + l2) * cos(theta) + v1(1);
xzplane.y = zeros(1,numel(xzplane.x));
xzplane.z = (l1 + l2) * sin(theta) + v1(3);

% For the yz plane starting at the shoulder
yzplane.x = zeros(1,numel(xzplane.x));
yzplane.y = (l1 + l2) * cos(theta) + v1(2);
yzplane.z = (l1 + l2) * sin(theta) + v1(3);

% Plot 
plot3(x1, y1, z1, '-b', ...
      x2, y2, z2, '-r', ...
      x3, y3, z3, '-c', ...
      xS, yS, zS, '-k', ...
      xyplane.x, xyplane.y, xyplane.z, 'k:', ...
      xzplane.x, xzplane.y, xzplane.z, 'k:', ...
      yzplane.x, yzplane.y, yzplane.z, 'k:', 'LineWidth', 2);

% Hold on to objects in the axes
hold on;
% Lock aspect ratios equal
axis equal;
% Put a box around axes
box on;
% Put gridlines on the figure
grid on;
% Set axes limits
axis([-2 2 -2 2 0 3]);
% Add title and axes to the figure
xlabel('X'); ylabel('Y'); zlabel('Z');
title({'Live position of 3-link-arm'});
view(37.5, 30);            % Rotate the plot to 'face forward'
end