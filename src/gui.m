javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

pp = PacketProcessor(7);
csv = 'values.csv';

% Create global values for the link lengths
global l1 l2 l3

% We need a fresh list of angles every time, or else the plot will not work 
delete 'values.csv'; delete 'armPos.csv';

% Set initial KP, KI, KD for joints 0, 1, and 2
gains = zeros(15, 1, 'single');
% kp, ki, kd for joint 0
gains(1) = 0.0025;
gains(2) = 0;
gains(3) = 0.015;
% kp, ki, kd for joint 1
gains(4) = 0.003;
gains(5) = 0.0005;
gains(6) = 0.03;
% kp, ki, kd for joint 2
gains(7) = 0.003;
gains(8) = 0;
gains(9) = 0.001;

% Set the PID gains using the packet processor
tic
pp.command(39, gains);
toc

%Create an array of 32 bit floaing point zeros to load an pass to the
%packet processor
values = zeros(15, 1, 'single');
%Fill the PID control command packet with inital values:
% Send setpoint for joint 0 in raw encoder ticks, plus velocity and
% torque targets
% Position ranges from -980 to 1250
values(1) = 0;
values(2) = 0;
values(3) = 0;
% Send setpoint for joint 1 in raw encoder ticks, plus velocity and
% torque targets
% Position ranges from -200 to 1000
values(4) = 0;
values(5) = 0;
values(6) = 0;
% Send setpoint for joint 2 in raw encoder ticks, plus velocity and
% torque targets
% Position ranges from -330 to 2400
values(7) = 0;
values(8) = 0;
values(9) = 0;

% Create the gui
guiInitializer;

setGlobalLinks(1, 1, 1);

% Loop forever to continuously plot arm
sentinel = 0;
while sentinel > 0
    % Process command, capture returning values
    tic
    returnValues = pp.command(38, values);
    toc
    
    pause(0.1);
    % Add the current returned values to the values .csv file
    dlmwrite(csv, transpose(returnValues), '-append');
    
    % Take returned encoder ticks and translate to degrees
    q0 = 0 - (returnValues(1) / 12);
    q1 = (returnValues(4) / 12);
    q2 = 0 - (returnValues(7) / 12);
    
    % Calculate the position of the elbow, 3x1 vector
    posElbow = eCoordinate(getl1, getl2, q0, q1);
    % Calculate the position of the tool tip, 3x1 vector
    posToolTip = pCoordinate(getl1, getl2, getl3, q0, q1, q2 + 90);
    % Combine coordinates of elbow and tip to create one vector for csv
    posArm = [posElbow posToolTip];
    % Write the cartesian coordinates of arm to csv file
    dlmwrite('armPos.csv',posArm,'-append','delimiter',' ');
    
    % Clear the live link plot
    clf;
    % Plot the link in real time using trig for arm positions
    threeLinkPlot(getl1, getl2, posElbow, posToolTip);
end

pp.shutdown()
clear java;


% GUI creation function
function guiInitializer
% GUI Select a data set from the pop-up menu, then
% click one of the plot-type push buttons. Clicking the button
% plots the selected data in the axes.

% Create and then hide the UI as it is being constructed
f = figure('Visible','off','Position',[360,500,900,570]);

% Construct the components
hstop   = uicontrol('Style','pushbutton',...
             'String','Stop','Position',[475,220,70,25],...
             'Callback',{@surfbutton_Callback});
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
ha = axes('Units','pixels','Position',[50,145,400,400]);
% Align components along their centers
align([hstop, htext, hviewController], 'Center','None');

% Initialize the UI.
% Change units to normalized so components resize automatically.
f.Units = 'normalized';
ha.Units = 'normalized';
hstop.Units = 'normalized';
% hmesh.Units = 'normalized';
% hcontour.Units = 'normalized';
htext.Units = 'normalized';
hviewController.Units = 'normalized';


% Create a dummy plot in the axes.
threeLinkPlot(1, 1, eCoordinate(1, 1, 0, 0),...
    pCoordinate(1, 1, 1, 0, 0, 0 + 90));

% Assign a name to appear in the window title.
f.Name = 'Arm Controller GUI';

% Move the window to the center of the screen.
movegui(f,'center')

% Make the UI visible.
f.Visible = 'on';
end

%  Pop-up menu callback. Read the pop-up menu Value property to
%  determine which view is currently selected by the user and make this the
%  current view. This callback automatically has access to 
%  current_data because this function is nested at a lower level.
function popup_menu_Callback(source) 
   % Determine the selected data set.
   str = source.String;
   val = source.Value;
   % Set current data to the selected data set.
   switch str{val}
   case 'Iso' % User selects Peaks.
      view(37.5, 30);
   case 'Top' % User selects Membrane.
      view(0,90);
   case 'Right' % User selects Sinc.
      view(90,0);
   case 'Front' % User selects
      view(0,0);
   end
end

% Set the values of the links
function setGlobalLinks(val1, val2, val3)
global l1 l2 l3
l1 = val1;
l2 = val2;
l3 = val3;
end

% Get the value of link 1
function L = getl1
global l1
L = l1;
end

% Get the value of link 2
function L = getl2
global l2
L = l2;
end

% Get the value of link 3
function L = getl3
global l3
L = l3;
end

% Push button callbacks. Each callback plots current_data in the
% specified plot type.
function stopbutton_Callback 
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
