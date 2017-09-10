function circlePlot(x, y, z, r)

hold on;

theta = 0:pi/50:2*pi;

xunit = r * cos(theta) + x;
yunit = r * sin(theta) + y;
zunit = zeros(1,numel(xunit));

figure
plot3(xunit, yunit, zunit,':');
hold off;
end