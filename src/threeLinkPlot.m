% Function threeLinkPlot takes the lengths of link 1 and 2,
% and the position of the elbow and tool tip to calculate the position
% of a 3-DOF arm with link lengths defined below.

% This function require function pCoordinate.m
function threeLinkPlot(l1, l2, p2, p3)

% Origin point, 3x1
p0 = [0; 0; 0];

% First link point, 3x1
p1 = [0; 0; 1];
% Vectors from p0 to p1 in x, y, z
x1 = [p0(1) p1(1)];
y1 = [p0(2) p1(2)];
z1 = [p0(3) p1(3)];

% Second link
% Vectors from p1 to p2 in x, y, z
x2 = [p1(1) p2(1)];
y2 = [p1(2) p2(2)];
z2 = [p1(3) p2(3)];

% Third link
% Vectors from p2 to p3 in x, y, z
x3 = [p2(1) p3(1)];
y3 = [p2(2) p3(2)];
z3 = [p2(3) p3(3)];

% Shadow link
% Projection of links 2 and 3 onto xy plane
xS1 = [p0(1) p2(1)];
yS1 = [p0(2) p2(2)];
zS1 = [p0(3) p0(3)];
xS2 = [p2(1) p3(1)];
yS2 = [p2(2) p3(2)];
zS2 = [p0(3) p0(3)];

% Create a circle showing the range of the arm
theta = 0:pi/50:2*pi;

% For the xy plane starting at the shoulder
xyplane.x = (l1 + l2) * cos(theta) + p1(1);
xyplane.y = (l1 + l2) * sin(theta) + p1(2);
xyplane.z = ones(1,numel(xyplane.x));

% For the xz plane starting at the shoulder
xzplane.x = (l1 + l2) * cos(theta) + p1(1);
xzplane.y = zeros(1,numel(xzplane.x));
xzplane.z = (l1 + l2) * sin(theta) + p1(3);

% For the yz plane starting at the shoulder
yzplane.x = zeros(1,numel(xzplane.x));
yzplane.y = (l1 + l2) * cos(theta) + p1(2);
yzplane.z = (l1 + l2) * sin(theta) + p1(3);

% Plot 
plot3(x1, y1, z1, '-b', ...
      x2, y2, z2, '-r', ...
      x3, y3, z3, '-c', ...
      xS1, yS1, zS1, '-k', ...
      xS2, yS2, zS2, '-k', ...
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