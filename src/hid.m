% TODO : Update link lengths through the code to do accurate error calcs
javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

%% Set up variables and file names
runtime = 15;

pp = PacketProcessor(7);
csv = 'values.csv';

% Initialize camera
if(~exist('cam','var'))
    cam = webcam('USB 2.0 Camera');
end

% Make an empty plot
createPlot;

% Set values for the lengths of link 1, 2, and 3
l1 = 20;
l2 = 17;
l3 = 20;

% Create the xyz position array
xyzPos = [];
qp = [0 0 0];
pointMatrix = [17 0 40; 20 16 6; 20 -16 6; 17 0 40];
% we need a fresh list of angles every time, or else the plot will not work 
delete 'values.csv'; delete 'armPos.csv'; delete 'pipPos.csv';

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

%% Begin program loop
previoustime = 0;
point = 1;
genesis = tic;
timeinterval = 0;
while 1
    
% Everything will break if we don't do a dlmwrite
     dlmwrite('pipPos.csv',[0 0 0],'-append','delimiter',' ');
     tic
     %Process command and print the returning values
     returnValues = pp.command(38, values);
     toc
     
     pause(0.1) %timeit(returnValues)
     dlmwrite(csv, transpose(returnValues), '-append');     
     
     % Take encoder ticks and translate to degrees
     q(1) = 0 - (returnValues(1) / 12);
     q(2) = (returnValues(4) / 12);
     q(3) = 0 - (returnValues(7) / 12);
     qp = [qp; q(1), q(2), q(3)];
     % Calculate the position of the elbow, 3x1 vector
     posElbow = eCoordinate(l1, l2, q(1), q(2));
     % Calculate the position of the tool tip, 3x1 vector
     posToolTip = pCoordinate(l1, l2, l3, q(1), q(2), q(3) + 90);
     % Combine coordinates of elbow and tip to create one vector for csv
     posArm = [posElbow posToolTip];
     % Write the cartesian coordinates of arm to csv file
     dlmwrite('armPos.csv',posArm,'-append','delimiter',' ');

     % Clear the live link plot
     clf;
     
     newSetpoint = invPosKinematics(pointMatrix(point, :));
     values(1) = newSetpoint(1) * 12;
     values(4) = newSetpoint(2) * 12;
     values(7) = newSetpoint(3) * 12;

     % Calculate the transformation matrix of the arm
     TM = forPosKinematics(q(1), q(2), -(q(3)+90));
     % Create the rotation matrix out of the transformation matrix
     RM = [TM(1,1),TM(1,2),TM(1,3);...
           TM(2,1),TM(2,2),TM(2,3);...
           TM(3,1),TM(3,2),TM(3,3)];
     % Calculate the transpose of the rotation matrix
     RMt = transpose(RM);
     % Create a vector of just the tip position
     TP = [TM(1,4);TM(2,4);TM(3,4)];
     
     xyzPos = [xyzPos; transpose(TP)];
     timer = toc;
     timeinterval = [timeinterval(1,:) previoustime+timer];
     previoustime = previoustime + timer;
     % Plot the link in real time using transformation matrices for arm
     % positions
     threeLinkPlot(l1, l2, posElbow, TP);
     
     if(returnValues(10) == 1 && returnValues(11) == 1 && returnValues(12) == 1)
         point = point + 1;
         pause(0.5);
         if(point > size(pointMatrix, 1))
             break;
         end
     end

     % if the total elapsed time is greater than desired, end the loop
     if(toc(genesis) > runtime) 
         break;
     end
end

%% Clean up and do final plotting 

% Read in the angles from the CSV file and plot them
xEpos = [0 0 0];%dlmread('armPos.csv',' ',[0 0 15 0]);
yEpos = [0 0 0];%dlmread('armPos.csv',' ',[0 1 15 1]);
zEpos = [0 0 0];%dlmread('armPos.csv',' ',[0 2 15 2]);
xTpos = xyzPos(:,1); %dlmread('armPos.csv',' ',[0 3 15 3]);
yTpos = xyzPos(:,2); %dlmread('armPos.csv',' ',[0 4 15 4]);
zTpos = xyzPos(:,3); %dlmread('armPos.csv',' ',[0 5 15 5]);
xPpos = [0 0 0];%(dlmread('pipPos.csv',' ',[0 0 19 0]) + 20) / 20;
yPpos = [0 0 0];%dlmread('pipPos.csv',' ',[0 1 19 1]) / 17;
zPpos = [0 0 0];%dlmread('pipPos.csv',' ',[0 2 19 2]);
s = transpose(qp);
pathPlot(xEpos, yEpos, zEpos, xTpos, yTpos, zTpos, xPpos, yPpos, zPpos);
%plot(timeinterval, s(1,:),timeinterval, s(2,:),'--', timeinterval, s(3,:),'p:');
%grid on;
pp.shutdown()
clear('cam');
clear java;