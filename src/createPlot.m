function createPlot
%% Create a figure environment
% Open a figure environment
%figure();
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