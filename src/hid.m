javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;


pp = PacketProcessor(7);
csv = 'values.csv';

%Create an array of 32 bit floaing point zeros to load an pass to the
%packet processor
values = zeros(15, 1, 'single');

createPlot;

% we need a fresh list of angles every time, or else the plot will not work
delete 'angles.csv';

% This loop runs for 10 seconds and iterates 40 times
for k=1:40
     %Create PID control command packet:
     % Send setpoint for joint 0 in raw encoder ticks, plus velocity and
     % torque targets
     values(1) = -350;
     values(2) = 400;
     values(3) = 200;
     % Send setpoint for joint 0 in raw encoder ticks, plus velocity and
     % torque targets
     values(4) = 300;
     values(5) = 450;
     values(6) = 230;
     % Send setpoint for joint 0 in raw encoder ticks, plus velocity and
     % torque targets
     values(7) = 100;
     values(8) = 800;
     values(9) = 150;
     
     tic
     %Process command and print the returning values
     returnValues = pp.command(38, values);
     toc
     disp('sent');
     disp(values);
     disp('got');
     disp(returnValues);
     
     % Byte Array Structure: 64 bytes
     %  4-byte command identifier
     %  Link 0 Position
     %  Link 0 Velocity
     %  Link 0 Torque
     %  Link 1 Position
     %  Link 1 Velocity
     %  Link 1 Torque
     %  Link 2 Position
     %  Link 2 Velocity
     %  Link 2 Torque
     %  empty
     %  empty
     %  empty
     %  empty
     %  empty
     %  empty
     
     pause(0.1) %timeit(returnValues)
     dlmwrite(csv, transpose(returnValues), '-append');     
     q0 = 0 - (returnValues(1) / 12);
     q1 = 0 - (returnValues(4) / 12);
     q2 = 0 - (returnValues(7) / 12);
%      dlmwrite('angles.csv',val,'-append','delimiter',' ')
     threeLinkPlot(q0, q1, q2);
     %pause(0.25);
 end
pp.shutdown()
clear java;
% Clear the live link plot
clf

% Read in the angles from the CSV file and plot them
% a = csvread('angles.csv');
% must generate a vector of time values, every quarter of a second
% t = linspace(0, 10, 40);

% Create a plot for the position of the link over time
% axes;                   % Add axes to the figure
% hold on;                % Hold on to objects in the axes
% box on;                 % Put a box around axes
% grid on;                % Put gridlines on the figure
% axis([0 10 -180 180]);  % Set axes limits
% title({'Angle of link over time'}); % Add title to the figure
% plot(t, a);             % Plot the position of the arm over time

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Load the xml file
% xDoc = xmlread('seaArm.xml');
% %All Arms
% allAppendage =xDoc.getElementsByTagName('appendage');
% %All walking legs
% %allAppendage =xDoc.getElementsByTagName('leg');
% %All drivabel wheels
% %allAppendage =xDoc.getElementsByTagName('drivable');
% %All steerable wheels
% %allAppendage =xDoc.getElementsByTagName('steerable');
% %Grab the first appendage
% appendages = allAppendage.item(0);
% %all the D-H parameter tags
% allListitems = appendages.getElementsByTagName('DHParameters');
% %Load the transfrom of home to the base of the arm
% baseTransform = appendages.getElementsByTagName('baseToZframe').item(0);
% %Print all the values
% printTag(baseTransform,'x');
% printTag(baseTransform,'y');
% printTag(baseTransform,'z');
% printTag(baseTransform,'rotw');
% printTag(baseTransform,'rotx');
% printTag(baseTransform,'roty');
% printTag(baseTransform,'rotz');
% % Print D-H parameters
% for k = 0:allListitems.getLength-1
%    thisListitem = allListitems.item(k);
%    fprintf('\nLink %i\n',k);
%    % Get the label element. In this file, each
%    % listitem contains only one label.
%    printTag(thisListitem,'Delta');
%    printTag(thisListitem,'Theta');
%    printTag(thisListitem,'Radius');
%    printTag(thisListitem,'Alpha');
% end
% % Get the value stored in a tag
% function value = tagValue(thisListitem,name)
%    % listitem contains only one label.
%    thisList = thisListitem.getElementsByTagName(name);
%    thisElement = thisList.item(0);
%    data  = thisElement.getFirstChild.getData;
%    value = str2double(data);
% end
% %Print out the tag name with its value
% function printTag(thisListitem,name)
%    data  = tagValue(thisListitem,name);
%    fprintf('%s \t%f\n',name,data);
% end