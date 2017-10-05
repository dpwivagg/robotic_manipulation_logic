% GUI Select a data set from the pop-up menu, then
% click one of the plot-type push buttons. Clicking the button
% plots the selected data in the axes.

import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

pp = PacketProcessor(7);
csv = 'values.csv';

delete 'values.csv';

%% KP Tuning Parameters
% Set KP, KI, KD for joints 0, 1, and 2 [P1;I1;D1;P2;I2;D2;P3;I3;D3]
gains = [0.0025; 0; 0.015; 0.003; 0.0005; 0.03; 0.003; 0; 0.001;...
    0;0;0;0;0;0];
% Set the PID gains using the packet processor
tic
pp.command(39, gains);
toc

%% Initial values for position command
% Set initial PID setpoints
values = zeros(15, 1, 'single');
% Position joint 0 ranges from -980 to 1250
% Position joint 1 ranges from -200 to 1000
% Position joint 2 ranges from -330 to 2400
tic
pp.command(38, values);
toc

%% Create the GUI
% Create and then hide the UI as it is being constructed
f = figure('Visible','off','Position',[360,500,900,570]);

% Construct the components
hstop   = uicontrol('Style','pushbutton',...
             'String','Stop','Position',[475,220,70,25],...
             'Callback',{@stopbutton_Callback});
% hmesh    = uicontrol('Style','pushbutton',...
%             'String','Mesh','Position',[315,180,70,25],...
%             'Callback',{@meshbutton_Callback});
% hcontour = uicontrol('Style','pushbutton',...
%             'String','Contour','Position',[315,135,70,25],...
%             'Callback',{@contourbutton_Callback});
htext    = uicontrol('Style','text',...
            'String','Select View','Position',[475,485,80,15]);
hviewController = uicontrol('Style','popupmenu',...
            'String',{'Iso','Top','Right','Front'},...
            'Position',[475,450,100,25],...
            'Callback',{@popup_menu_Callback});
% Add axes and define placement of live plot
ax = axes('Units','pixels','Position',[50,145,400,400]);
% Align components along their centers
align([hstop, htext, hviewController], 'Center','None');

% Initialize the UI.
% Change units to normalized so components resize automatically.
f.Units = 'normalized';
ax.Units = 'normalized';
hstop.Units = 'normalized';
% hmesh.Units = 'normalized';
% hcontour.Units = 'normalized';
htext.Units = 'normalized';
hviewController.Units = 'normalized';

% Assign a name to appear in the window title.
f.Name = 'Arm Controller GUI';

% Move the window to the center of the screen.
movegui(f,'center')

% Make the UI visible.
f.Visible = 'on';

%% Other Stuff

% Set values for the lengths of link 1, 2, and 3
global links vParam
links = [20 17 20];
vParam = [-37.5 30];
% set global joint angle values
global q 
q = [];
torque = [];
runtime = 15;
genesis = tic;
sentinel = 1;
while sentinel
    tic
    %Process command and print the returning values
    returnValues = pp.command(38, values);
    toc
    pause(0.1)
    dlmwrite(csv, transpose(returnValues), '-append');
    
    % Take encoder ticks and translate to degrees
    q(1) = 0 - (returnValues(1) / 12);
    q(2) = (returnValues(4) / 12);
    q(3) = -((0 - (returnValues(7) / 12))+90);
    
    % Calculate force vector at tip from torques at joints
    forceTip = tipforcevector([returnValues(3); returnValues(6); returnValues(9)]);
    % Calculate the magnitude for the tip force in each direction
    magFT(1) = sqrt(forceTip(1)^2);
    magFT(2) = sqrt(forceTip(2)^2);
    magFT(3) = sqrt(forceTip(3)^2);
    % Create a unit vector of the tip force
    uForceTip(1) = forceTip(1)/magFT(1);
    uForceTip(2) = forceTip(2)/magFT(2);
    uForceTip(3) = forceTip(3)/magFT(3);
    % Scale the unit vector by 10 for plotting
    uForceTip(1) = uForceTip(1)*10;
    uForceTip(2) = uForceTip(2)*10;
    uForceTip(3) = uForceTip(3)*10;
        
    % Calculate the transformation matrix of the arm
    TM = forPosKinematics(1);
    
    % Create a vector of just the tip position
    TP = [TM(1,4);TM(2,4);TM(3,4)];
    
    % Create a vector of just the elbow
     TMe = forPosKinematics(0);
     TPe = [TMe(1,4);TMe(2,4);TMe(3,4)];
    
    %axes(ax);
    threeLinkPlot(ax,TPe,TP,uForceTip);
    
    % if the total elapsed time is greater than desired, end the loop
    if(toc(genesis) > runtime) 
        break;
    end
end

pp.shutdown()
clear java;


%  Pop-up menu callback. Read the pop-up menu Value property to
%  determine which view is currently selected by the user and make this the
%  current view. This callback automatically has access to 
%  current_data because this function is nested at a lower level.
function popup_menu_Callback(source, event) 
   % Determine the selected data set.
   str = source.String;
   val = source.Value;
   % Set current data to the selected data set.
   switch str{val}
   case 'Iso' % User selects an isometric view
      setGlobalvParam(37.5,30);
   case 'Top' % User selects a top view
      setGlobalvParam(0,90);
   case 'Right' % User selects a right side view
      setGlobalvParam(0,0);
   case 'Front' % User selects
      setGlobalvParam(90,0);
   end
end

% Push button callbacks. Each callback plots current_data in the
% specified plot type.
function stopbutton_Callback(source, event)
    % Display surf plot of the currently selected data.
     sentinel = 0;
end

% function meshbutton_Callback(source,eventdata) 
%     % Display mesh plot of the currently selected data.
%      mesh(current_data);
% end
% 
% function contourbutton_Callback(source,eventdata) 
%     % Display contour plot of the currently selected data.
%      contour(current_data);
% end