% linkPlot accepts an angle in degrees and 
% plots the position of the link
function linkPlot(q)

% Convert from degrees to radians
q = q * 0.017;

% Calculate cartesian coordinates for link
X = cos(q);
Y = sin(q);

% Plot the link
plot(X,Y,'LineWidth',2,'MarkerSize',3);

% Update the plot
drawnow;

