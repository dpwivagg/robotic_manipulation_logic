javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

pp = PacketProcessor(7);
csv = 'values.csv';

% Make an empty plot
createPlot;

% We need to cycle through 3 states before terminating the program
state = 0;

% Set values for the lengths of link 1, 2, and 3
l1 = 1;
l2 = 1;
l3 = 1;

% Set KP, KI, KD for joints 0, 1, and 2
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
values(4) = 760;
values(5) = 0;
values(6) = 0;
% Send setpoint for joint 2 in raw encoder ticks, plus velocity and
% torque targets
% Position ranges from -330 to 2400
values(7) = -330;
values(8) = 0;
values(9) = 0;

% we need a fresh list of angles every time, or else the plot will not work
delete 'xpos.csv'; delete 'ypos.csv'; delete 'zpos.csv'; 
delete 'values.csv';

% This loop terminates after a few seconds to ensure the program ends in
% case of an error in the firmware
for k=1:20
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
     
     % Take encoder ticks and translate to degrees
     q0 = 0 - (returnValues(1) / 12);
     q1 = (returnValues(4) / 12);
     q2 = 0 - (returnValues(7) / 12);
     
     % Calculate the position of the tool tip
     posToolTip = pCoordinate(l1, l2, l3, q0, q1, (q2 + 90));
     
     dlmwrite('xpos.csv',posToolTip(1),'-append','delimiter',' ')
     dlmwrite('ypos.csv',posToolTip(2),'-append','delimiter',' ')
     dlmwrite('zpos.csv',posToolTip(3),'-append','delimiter',' ')

     % Clear the live link plot
     clf;
     threeLinkPlot(l1, l2,...
                   eCoordinate(l1, l2, q0, q1),...
                   pCoordinate(l1, l2, l3, q0, q1, (q2 + 90)));
%      This is some potential code for stopping the robot once it reaches
%      the setpoint
     if(returnValues(10)==1 && returnValues(11)==1 && returnValues(12) == 1)
         if(state == 0)
             values(4) = 972;
             values(7) = -165;
             state = 1;
             pause(1);
         
         elseif(state == 1)
             values(4) = 350;
             values(7) = 410;
             state = 2;
             pause(1);
         
         elseif(state == 2)
             values(4) = 760;
             values(7) = -343;
             state = 3;
             pause(1);
         
         elseif(state == 3)
            break
         end
%         break
     end
 end
 
pp.shutdown()
clear java;

% Read in the angles from the CSV file and plot them
xpos = csvread('xpos.csv');
ypos = csvread('ypos.csv');
zpos = csvread('zpos.csv');

pathPlot(xpos, ypos, zpos);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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