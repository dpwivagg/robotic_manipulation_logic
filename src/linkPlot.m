% linkPlot accepts an angle in degrees and 
% plots the position of the link
function linkPlot(q)

% Calculate cartesian coordinates for link
X = cosd(q);
Y = sind(q);

% Clear the figure
clf

% Plot the link
plotv(X,Y,'-o');

% Update the plot
drawnow;

