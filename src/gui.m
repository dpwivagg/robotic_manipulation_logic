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
% "Stop" button - stops the loop
hstop   = uicontrol('Style','pushbutton',...
             'String','Stop','Position',[475,220,70,25],...
             'Callback',{@stopbutton_Callback});

% Set Point fields - Allows the user to enter an [x y z] setpoint
hsetpointX   = uicontrol('Style','edit',...
             'String','0','Position',[132,50,70,25],...
             'Callback',{@setpointX_Callback});
hsetpointY   = uicontrol('Style','edit',...
             'String','0','Position',[212,50,70,25],...
             'Callback',{@setpointY_Callback});
hsetpointZ   = uicontrol('Style','edit',...
             'String','0','Position',[292,50,70,25],...
             'Callback',{@setpointZ_Callback});
         
% "Go" button - Sends the robot to the setpoint specified in the set point
% fields, defaults to [0 0 0]
hgoToSetpoint = uicontrol('Style','pushbutton',...
             'String','Go XYZ','Position',[212,80,70,25],...
             'Callback',{@goToSetpoint_Callback});

% "Select View" text field - describes viewController menu         
htext    = uicontrol('Style','text',...
            'String','Select View','Position',[475,485,80,15]);

% View controller drop down menu - gives the user four options to view the
% live 3D plot: top, side, front, and isometric views
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
global sentinel state
sentinel = 1;
state = 1;

while getSentinel()
    switch state
        % This is the "pause" state
        case 0
            % Do nothing
        
        % Home state
        case 1
            tic
            %Process command and print the returning values
            returnValues = pp.command(38, values);
            toc
            pause(0.1)
            dlmwrite(csv, transpose(returnValues), '-append');
            % Take encoder ticks and translate to degrees
            setJointValues(0 - (returnValues(1) / 12),...
                          (returnValues(4) / 12),...
                          (0 - (returnValues(7) / 12)));
                      
            % Calculate the transformation matrix of the arm from base frame
            % to tip frame
            TM = forPosKinematics(1);
            % Calculate the transformation matrix of the arm from base frame
            % to elbow frame
            TMe = forPosKinematics(0);
            
            % Create a vector of the tip position
            TP = [TM(1,4);TM(2,4);TM(3,4)];
            % Create a vector of the elbow position
            TPe = [TMe(1,4);TMe(2,4);TMe(3,4)];
            
            % Plot the position of the arm
            livePlot(ax,TPe,TP,TP);
        
        % Go to XYZ position set by user
        case 2
    end
%     tic
%     %Process command and print the returning values
%     returnValues = pp.command(38, values);
%     toc
%     pause(0.1)
%     dlmwrite(csv, transpose(returnValues), '-append');
%     
%     % Take encoder ticks and translate to degrees
%     q(1) = 0 - (returnValues(1) / 12);
%     q(2) = (returnValues(4) / 12);
%     q(3) = -((0 - (returnValues(7) / 12))+90);
%     
%     % Calculate force vector at tip from torques at joints
%     forceTip = tipforcevector([returnValues(3); returnValues(6); returnValues(9)]);
%         
%     % Calculate the transformation matrix of the arm
%     TM = forPosKinematics(1);
%     
%     % Create a vector of just the tip position
%     TP = [TM(1,4);TM(2,4);TM(3,4)];
%     
%     % Create a vector of just the elbow
%      TMe = forPosKinematics(0);
%      TPe = [TMe(1,4);TMe(2,4);TMe(3,4)];
%     
%     %axes(ax);
%     threeLinkPlot(TPe,TP,forceTip);
    
    % if the total elapsed time is greater than desired, end the loop
    if(toc(genesis) > runtime) 
        break;
    end
end

pp.shutdown()
clear java;


%  Pop-up menu callback. Read the pop-up menu Value property to
%  determine which view is currently selected by the user and make this the
%  current view. The view preference is updated by the setGlobalvParam
%  function.
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
   case 'Front' % User selects a front view
      setGlobalvParam(90,0);
   end
end

% Push button callback. When the user presses this button, stop the loop
% from executing and clear java to shut down the arm.
function stopbutton_Callback(source, event)
    % Display surf plot of the currently selected data.
    setSentinel(0);
end

% Text entry fields. The user can use these fields to enter an [X Y Z]
% setpoint of their choice. When the user presses the "Go XYZ" button the
% desired setpoints are given to the arm. 
function setpointX_Callback(source,event) 
    % Get the value entered by the user
    x = get(source,'string');
end

function setpointY_Callback(source,event) 
    % Get the value entered by the user
    y = get(source,'string');
end

function setpointZ_Callback(source,event) 
    % Get the value entered by the user
    z = get(source,'string');
end

% Push button callback. The user presses the "Go XYZ" button after entering
% an [X Y Z] setpoint. The values entered for the desired setpoint are
% checked to make sure they are within range. If they are within range, the
% robot is sent to the desired setpoint. Otherwise, an error message
% appears
function goToSetpoint_Callback(source,event) 
    % Display contour plot of the currently selected data.
     contour(current_data);
end