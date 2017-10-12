javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

%% Set up variables and file names
runtime = 40;

stateUpdate = 1;

pp = PacketProcessor(7);
csv = 'values.csv';

% Initialize camera
if(~exist('cam','var'))
    cam = webcam('USB 2.0 Camera');
end

% Set values for the lengths of link 1, 2, and 3
global links
links = [20 17 20];
global  vParam
vParam = [-37.5 30];
% set global joint angle values
global q 
q = [];
ax = axes('Units','pixels','Position',[50,145,400,400]);
ax.Units = 'normalized';
% Create the xyz position array to store values
xyzPos = [];
plotValues = [];

% we need a fresh list of angles every time, or else the plot will not work 
delete 'values.csv'; delete 'armPosetpoints.csv';

%% KP Tuning Parameters
% Set KP, KI, KD for joints 0, 1, and 2 [P1;I1;D1;P2;I2;D2;P3;I3;D3]
gains = [0.0025; 0; 0.015; 0.002; 0.0005; 0.03; 0.001; 0; 0.0015;...
    0;0;0;0;0;0];
% Set the PID gains using the packet processor
tic
pp.command(39, gains);
toc

%% Open the servo
servoPacket = zeros(15, 1, 'single');
% 0 = open
% 1 = closed
servoPacket(1) = 1;
tic
pp.command(48, servoPacket);
toc

desiredSetpoints = [20 0 37; 15 5 3; 25 10 10];% 20 -10 20; 17 15 15; 15 10 10; 20 5 10; 15 -5 10; 37 0 20; 30 5 20];


%% Initial values for position command
% Set initial PID setpoints
values = zeros(15, 1, 'single');
% Position joint 0 ranges from -980 to 1250
% Position joint 1 ranges from -200 to 1000
% Position joint 2 ranges from -330 to 2400

%% Begin program loop
timep = [];
point = 1;
genesis = tic;
uForceTipa = [];
magforce=[];
state = 1;
% State 1: Find objects and travel to one
% State 2: Pick up an object and travel to weighing position
% State 3: Weigh the object and travel to sorting position
% State 3: Drop the object

flag = 0;
while 1
     for i = 1:2
         flag = 0;
         pointMatrix = findTotalTrajectory([desiredSetpoints(i,:);desiredSetpoints((i+1),:)],0.2);
         while(flag == 0)
             tic
             %Process command and print the returning values
             returnValues = pp.command(38,values);
             toc

             pause(0.1) %timeit(returnValues)

             plotValues = [plotValues; transpose(returnValues)];
             % Take encoder ticks and translate to degrees
             q(1) = 0 - (returnValues(1) / 12);
             q(2) = (returnValues(4) / 12);
             q(3) = (0 - (returnValues(7) / 12));

             % Determine the force vector at the tip
             uForceTip = tipforcevector([returnValues(3); returnValues(6); returnValues(9)]);
             uForceTipa = [uForceTipa; uForceTip(1), uForceTip(2), uForceTip(3)];
             magforce =[magforce; sqrt(uForceTip(1)^2+uForceTip(2)^2+uForceTip(3)^2)];
             timep = [timep; toc];
             % Clear the live link plot
             clf;

             newSetpoint = invPosKinematics(pointMatrix(point, :));
             values(1) = newSetpoint(1) * 12;
             values(4) = newSetpoint(2) * 12;
             values(7) = newSetpoint(3) * 12;

             % Calculate the transformation matrix of the arm and get tip position
             TM = forPosKinematics(1);
             TP = [TM(1,4);TM(2,4);TM(3,4)];  

             % Create a vector of the elbow position
             TMe = forPosKinematics(0);
             TPe = [TMe(1,4);TMe(2,4);TMe(3,4)];

             % Store the tip position for plotting at the end
             xyzPos = [xyzPos; transpose(TP)];   

             % Plot the link in real time using transformation matrices for arm
             % positions
             threeLinkPlot(TPe,TP,uForceTip);


             if(returnValues(10) == 1 && returnValues(11) == 1 && returnValues(12) == 1)
                 point = point + 1;
                 if(point > size(pointMatrix, 1))
                     flag = 1;
                     point = 1;
                     pause(5);
                 end
             end

         end
         
     end
      % if the total elapsed time is greater than desired, end the loop
             if(toc(genesis) > runtime) 
                 break;
             end
end

%% Clean up and do final plotting 
dlmwrite(csv, plotValues, '-append');    
% Read in the angles from the matrix and plot them
% pathPlot(xyzPos(:,1), xyzPos(:,2), xyzPos(:,3));
% subplot(3,1,1);
% plot(timep,xyzPos(:,1),'-r',timep,xyzPos(:,2),'-b',timep,xyzPos(:,1),'-g')
% grid on;
% subplot(3,1,2);
uForceTipa = transpose(uForceTipa);
plot(timep,uForceTipa(1,:),'-r',timep,uForceTipa(2,:),'-b',timep,uForceTipa(3,:),'-g')
% grid on;
% subplot(3,1,3);
% plot(timep,magforce)
grid on;
pp.shutdown()
clear('cam');
clear java;