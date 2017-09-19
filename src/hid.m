javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

%% Set up variables and file names
runtime = 20;

pp = PacketProcessor(7);
csv = 'values.csv';

% Initialize camera
cam = webcam('USB 2.0 Camera');

% Make an empty plot
createPlot;

% Set values for the lengths of link 1, 2, and 3
l1 = 1;
l2 = 1;
l3 = 1;

% we need a fresh list of angles every time, or else the plot will not work 
delete 'values.csv'; delete 'armPos.csv';

%% KP Tuning Parameters
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

%% Initial values for position command
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

%% Take a camera snapshot to start with
% take snapshot of workspace
img = snapshot(cam);

% crop enhance and change image and bring back centrioid cordinates
centroidpix = processImage(img);

% convert pixels to xy
[xcord,ycord] = mn2xy(centroidpix(1,1),centroidpix(1,2));
objposition = [xcord;ycord;0];

%% Set up initial velocity setpoint

% Calculate the transformation matrix of the arm
TM = forPosKinematics(0, 0, -90);

% Create a vector of just the tip position
TP = [TM(1,4);TM(2,4);TM(3,4)];

% Create desired velocity setpoints and direction
taskV1 = (objposition - TP);

%% Begin program loop
genesis = tic;
while 1
     
     tic
     %Process command and print the returning values
     returnValues = pp.command(38, values);
     toc
     
     pause(0.1) %timeit(returnValues)
     dlmwrite(csv, transpose(returnValues), '-append');     
     
     % Take encoder ticks and translate to degrees
     q0 = 0 - (returnValues(1) / 12);
     q1 = (returnValues(4) / 12);
     q2 = 0 - (returnValues(7) / 12);
     
     % Calculate the position of the elbow, 3x1 vector
     posElbow = eCoordinate(l1, l2, q0, q1);
     % Calculate the position of the tool tip, 3x1 vector
     posToolTip = pCoordinate(l1, l2, l3, q0, q1, q2 + 90);
     % Combine coordinates of elbow and tip to create one vector for csv
     posArm = [posElbow posToolTip];
     % Write the cartesian coordinates of arm to csv file
     dlmwrite('armPos.csv',posArm,'-append','delimiter',' ');

     % Clear the live link plot
     clf;
     
     % Calculate the transformation matrix of the arm
     TM = forPosKinematics(q0, q1, -(q2+90));
     % Create the rotation matrix out of the transformation matrix
     RM = [TM(1,1),TM(1,2),TM(1,3);...
           TM(2,1),TM(2,2),TM(2,3);...
           TM(3,1),TM(3,2),TM(3,3)];
     % Calculate the transpose of the rotation matrix
     RMt = transpose(RM);
     % Create a vector of just the tip position
     TP = [TM(1,4);TM(2,4);TM(3,4)];
     % Plot the link in real time using transformation matrices for arm
     % positions
     threeLinkPlot(l1, l2, posElbow, TP);
     
     % Calculate the inverse velocity kinematics
     jointV1 = double(invVelKinematics([3;-7;0], q0, q1, q2));
     % Create a new setpoint vector using the inverse velocity and elapsed
     % time
     newSetpoint = jointV1 * toc
     %newSetpoint = newSetpoint * (objposition - TP);
     values(1) = values(1) + newSetpoint(1)*12;
     values(4) = values(4) + newSetpoint(2)*12;
     values(7) = values(7) + newSetpoint(3)*12;
     dlmwrite('setpoints.csv',newSetpoint,'-append','delimiter',' ');
     
     % if the total elapsed time is greater than desired, end the loop
     if(toc(genesis) > runtime) 
         break;
     end
end
 
%% Clean up and do final plotting 
pp.shutdown()
clear('cam');
clear java;

% Read in the angles from the CSV file and plot them
xEpos = dlmread('armPos.csv',' ',[0 0 39 0]);
yEpos = dlmread('armPos.csv',' ',[0 1 39 1]);
zEpos = dlmread('armPos.csv',' ',[0 2 39 2]);
xTpos = dlmread('armPos.csv',' ',[0 3 39 3]);
yTpos = dlmread('armPos.csv',' ',[0 4 39 4]);
zTpos = dlmread('armPos.csv',' ',[0 5 39 5]);

pathPlot(xEpos, yEpos, zEpos, xTpos, yTpos, zTpos);