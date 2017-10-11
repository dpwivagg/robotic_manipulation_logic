% This function sets up the live 3D plot of the arm
function p = initLivePlot()
    % Update global parameters
    linkVal = [20 17 20];%getLinkValues();
    viewAzEl = [37.5,30];%getGlobalvParam();
    
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
    p = plot3(0,0,0, ...
              0,0,0, ...
              0,0,0, ...
              0,0,0, ...
              0,0,0, ...
              xyplane.x, xyplane.y, xyplane.z, ...
              xzplane.x, xzplane.y, xzplane.z, ...
              yzplane.x, yzplane.y, yzplane.z);
    
    set(p,prop_name,prop_values);
    drawnow();
    
    % Hold on to objects in the axes
    hold on;
    %annotation('arrow',xF,yF, zF);
    quiver3(0,0,0,0,0,0, 'LineWidth',2);
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