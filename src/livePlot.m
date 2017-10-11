% This function takes in the cartesian coordinates of the arm elbow and tip
% along with the taskspace forces at the tip and produces a live 3D plot
function livePlot(ax, p2, p3, forces)
    % Update global parameters
    linkVal = getLinkValues();
    viewAzEl = getGlobalvParam();
    
    % Clear axes
    cla(ax);
    
    % Origin point, 3x1
    p0 = [0; 0; 0];

    % First link point, 3x1
    p1 = [0; 0; linkVal(1)];
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
    xyplane.x = (linkVal(1) + linkVal(2)) * cos(theta);
    xyplane.y = (linkVal(1) + linkVal(2)) * sin(theta);
    xyplane.z = ones(1,numel(xyplane.x)) * linkVal(1);

    % For the xz plane starting at the shoulder
    xzplane.x = (linkVal(1) + linkVal(2)) * cos(theta);
    xzplane.y = zeros(1,numel(xzplane.x));
    xzplane.z = (linkVal(1) + linkVal(2)) * sin(theta) + linkVal(1);

    % For the yz plane starting at the shoulder
    yzplane.x = zeros(1,numel(xzplane.x));
    yzplane.y = (linkVal(1) + linkVal(2)) * cos(theta);
    yzplane.z = (linkVal(1) + linkVal(2)) * sin(theta) + linkVal(1);

    % Force vector at tip in direction of force
    xF = [p3(1) (forces(1))];
    yF = [p3(2) (forces(2))];
    zF = [p3(3) (forces(3))];
    
    prop_name(1) = {'LineWidth'};
    prop_name(2) = {'Color'};
    prop_name(3) = {'LineStyle'};
    
    % Three links of arm
    prop_values(1,1) = {3};
    prop_values(1,2) = {'b'};
    prop_values(1,3) = {'-'};
    prop_values(2,1) = {3};
    prop_values(2,2) = {'r'};
    prop_values(2,3) = {'-'};
    prop_values(3,1) = {3};
    prop_values(3,2) = {'c'};
    prop_values(3,3) = {'-'};
    
    % 'Shadow' links 
    prop_values(4,1) = {1};
    prop_values(4,2) = {'k'};
    prop_values(4,3) = {'-'};
    prop_values(5,1) = {1};
    prop_values(5,2) = {'k'};
    prop_values(5,3) = {'-'};
    
    % Circles of range
    prop_values(6,1) = {1};
    prop_values(6,2) = {'k'};
    prop_values(6,3) = {':'};
    prop_values(7,1) = {1};
    prop_values(7,2) = {'k'};
    prop_values(7,3) = {':'};
    prop_values(8,1) = {1};
    prop_values(8,2) = {'k'};
    prop_values(8,3) = {':'};
    
    % Plot the links
    p = plot3(x1,y1,z1, ...
              x2,y2,z2, ...
              x3,y3,z3, ...
              xS1,yS1,zS1, ...
              xS2,yS2,zS2, ...
              xyplane.x, xyplane.y, xyplane.z, ...
              xzplane.x, xzplane.y, xzplane.z, ...
              yzplane.x, yzplane.y, yzplane.z);
    
    set(p,prop_name,prop_values);
    drawnow();
    
    % Hold on to objects in the axes
    hold on;
    %annotation('arrow',xF,yF, zF);
    quiver3(xF(1),yF(1),zF(1),xF(2),yF(2),zF(2), 'LineWidth',2);
    % Lock aspect ratios equal
    axis equal;
    % Put a box around axes
    box on;
    % Put gridlines on the figure
    grid on;
    % Set axes limits
    axis([-40 40 -40 40 0 60]);
    % Add title and axes to the figure
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title({'Live position of 3-link-arm'});
    view(viewAzEl(1),viewAzEl(2));
end