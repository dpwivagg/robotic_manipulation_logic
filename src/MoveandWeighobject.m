%% Move to object and then return weight
% take in x and y cordiunates and packet processor 
function t = MoveandWeighobject(x,y,pp)

desiredSetpoints = [20 0 37; x y 3; 20 0 37];
pointMatrix = findTotalTrajectory(desiredSetpoints);
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
servoPacket(1) = 0;
tic
pp.command(48, servoPacket);
toc

%% Initial values for position command
% Set initial PID setpoints
% values = zeros(15, 1, 'single');
% Position joint 0 ranges from -980 to 1250
% Position joint 1 ranges from -200 to 1000
% Position joint 2 ranges from -330 to 2400

%% Begin program loop

point = 1;
genesis = tic;

while 1
    try
     tic
     %Process command and print the returning values
     returnValues = pp.command(38, values);
     toc
     
     pause(0.1) %timeit(returnValues)
    catch
        warning('Cant read returned values closing program');
        break;
    end
     returnedvalues = [returnedvalues; transpose(returnValues)];
     % Take encoder ticks and translate to degrees
     q(1) = 0 - (returnValues(1) / 12);
     q(2) = (returnValues(4) / 12);
     q(3) = 0 - (returnValues(7) / 12);
     % Get torque values from the packet
     uForceTip = tipforcevector([returnValues(3);returnValues(6);returnValues(9)]);
    
     % Clear the live link plot
     clf;
     
     newSetpoint = invPosKinematics(pointMatrix(point, :));
     values(1) = newSetpoint(1) * 12;
     values(4) = newSetpoint(2) * 12;
     values(7) = newSetpoint(3) * 12;

%      % Calculate the transformation matrix of the arm
%      TM = forPosKinematics(1);
%      % Create the rotation matrix out of the transformation matrix
%      RM = [TM(1,1),TM(1,2),TM(1,3);...
%            TM(2,1),TM(2,2),TM(2,3);...
%            TM(3,1),TM(3,2),TM(3,3)];
%      % Calculate the transpose of the rotation matrix
%      RMt = transpose(RM); 
%      
%      % Create a vector of just the tip position and elbow
%      TMe = forPosKinematics(0);
%      TPe = [TMe(1,4);TMe(2,4);TMe(3,4)];
%      TP = [TM(1,4);TM(2,4);TM(3,4)];  
      
     %xyzPos = [xyzPos; transpose(TP)];   
     %forcev = tipforcevector(torque);
     % Plot the link in real time using transformation matrices for arm
     % positions
     % threeLinkPlot(ax,TPe,TP,uForceTip);
        
     
     if(returnValues(10) == 1 && returnValues(11) == 1 && returnValues(12) == 1)
         point = point + 1;
         % These lines are for picking up the object (closing the servo)
         % They occur at points=47 because the pointMatrix is a very large
         % matrix with all of the setpoints along the trajectory.
         % Eventually we will be deleting this section when we have another
         % way to know when to close the gripper (I.E. state machine) but
         % for now 47 is a good enough estimate when we are testing.
         if(point == 47)
             servoPacket(1) = 1;
            tic
            pp.command(48, servoPacket);
            toc
         end
         if(point > size(pointMatrix, 1))
             break;
         end
     end

     % if the total elapsed time is greater than desired, end the loop
     if(toc(genesis) > runtime) 
         break;
     end
end
t=[returnValues(3);returnValues(6);returnValues(9)];
end
