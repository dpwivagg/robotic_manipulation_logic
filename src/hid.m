javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

%% Set up variables and file names
runtime = 60;

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
gains = [0.0025; 0; 0.015; 0.003; 0.0005; 0.03; 0.003; 0; 0.001;...
    0;0;0;0;0;0];
% Set the PID gains using the packet processor
tic
pp.command(39, gains);
toc

%% Open the servo
servoPacket = zeros(15, 1, 'single');
% 0 = open
% 1 = closed
servoPacket(1) = 0;
tic
pp.command(48, servoPacket);
toc

%% Initial values for position command
% Set initial PID setpoints
values = zeros(15, 1, 'single');
% Position joint 0 ranges from -980 to 1250
% Position joint 1 ranges from -200 to 1000
% Position joint 2 ranges from -330 to 2400

%% Begin program loop

point = 1;
genesis = tic;

state = 1;
% State 1: Find objects and travel to one
% State 2: Pick up an object and travel to weighing position
% State 3: Weigh the object and travel to sorting position
% State 3: Drop the object

while 1
    if(stateUpdate == 1)
        switch state
            case 1
                if(Imagefindandprocess('blue',cam) ~= 0)
                    [location(1),location(2)] = Imagefindandprocess('blue',cam);
                    xColorValue = 24;
                elseif(Imagefindandprocess('green',cam) ~= 0)
                    [location(1),location(2)] = Imagefindandprocess('green',cam);
                    xColorValue = 20;
                else %Object is yellow
                    [location(1),location(2)] = Imagefindandprocess('yellow',cam);
                    xColorValue = 16;
                end
                desiredSetpoints = [20 0 37; 23+location(1) location(2) 3];
                pointMatrix = findTotalTrajectory(desiredSetpoints);
                point = 1;
                stateUpdate = 0;
            case 2
                servoPacket(1) = 1;
                tic
                pp.command(48, servoPacket);
                toc
                 pointMatrix = flipud(pointMatrix);
                point = 1;
                state = 3;
            case 3
                magForceTip = sqrt(uForceTip(1)^2 + uForceTip(2)^2 + uForceTip(3)^2);
                if(magForceTip > 1)
                    desiredSetpoints = [20 0 37; xColorValue -16 3];
                else
                    desiredSetpoints = [20 0 37; xColorValue 16 3];
                end
                pointMatrix = findTotalTrajectory(desiredSetpoints);
                point = 1;
                stateUpdate = 0;
            case 4
                servoPacket(1) = 0;
                tic
                pp.command(48, servoPacket);
                toc
                state = 1;
            otherwise
                % If anything goes wrong, start over
                state = 1;
                point = 1;
        end
    end

     tic
     %Process command and print the returning values
     returnValues = pp.command(38, values);
     toc
     
     pause(0.1) %timeit(returnValues)

     plotValues = [plotValues; transpose(returnValues)];
     % Take encoder ticks and translate to degrees
     q(1) = 0 - (returnValues(1) / 12);
     q(2) = (returnValues(4) / 12);
     q(3) = (0 - (returnValues(7) / 12));
     
     % Determine the force vector at the tip
     uForceTip = tipforcevector([returnValues(3); returnValues(6); returnValues(9)]);

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
     threeLinkPlot(ax,TPe,TP,uForceTip);
        
     
     if(returnValues(10) == 1 && returnValues(11) == 1 && returnValues(12) == 1)
         point = point + 1;
         if(point > size(pointMatrix, 1))
             stateUpdate = 1;
             state = state + 1;
             point = 1;
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
pathPlot(xyzPos(:,1), xyzPos(:,2), xyzPos(:,3));

pp.shutdown()
clear('cam');
clear java;