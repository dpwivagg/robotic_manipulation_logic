function threeLinkPlot(q0, q1, q2)

l1 = 1;
l2 = 1;
l3 = 1;

% Origin
v0 = [0 0 0];

% First link
v1 = [0 0 1]
x1 = [v0(1) v1(1)];
y1 = [v0(2) v1(2)];
z1 = [v0(3) v1(3)];

% Second link
v2 = [(l2 * cosd(q1) * cosd(q0)) (l2 * cosd(q1) * sind(q0)) (l1 + l2 * sind(q1))];
x2 = [v1(1) v2(1)];
y2 = [v1(2) v2(2)];
z2 = [v1(3) v2(3)];

v3 = pCoordinate(q0, q1, (q2 + 90));
x3 = [v2(1) v3(1)];
y3 = [v2(2) v3(2)];
z3 = [v2(3) v3(3)];


plot3(x1, y1, z1, x2, y2, z2, x3, y3, z3);
% Hold on to objects in the axes
hold on;
% Lock aspect ratios equal
axis equal;
% % Put a box around axes
box on;
% % Put gridlines on the figure
grid on;
% % Set axes limits
axis(2*[-1 1 -1 1 0 1]);
% Add title to the figure
xlabel('X');
ylabel('Y');
zlabel('Z');
title({'Live position of 3-link-arm'});

end