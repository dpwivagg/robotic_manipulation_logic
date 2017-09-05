% linkPlot accepts an angle in degrees and 
% plots the position of the link
function linkPlot(q)

% Add axes to the figure
axes;
% Hold on to objects in the axes
hold on;
% Lock aspect ratios equal
axis equal;
% Put a box around axes
box on;
% Put gridlines on the figure
grid on;
% Set axes limits
axis(2*[-1 1 -1 1]);
% Add title to the figure
title({'Live position of 1-link-arm'});

% Calculate cartesian coordinates for link
X = cosd(q);
Y = sind(q);

m = [0 X; 0 Y];

% Plot the link
plotv(m,'-')

% Update the plot
drawnow;

% Turn hold off
hold off;

