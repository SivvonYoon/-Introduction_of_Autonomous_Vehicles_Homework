close all; clear; clc

addpath('../api')
% simRemoteApi.start(19999): simple test
% simRemoteApi.start(19999,1300,false,true) : synchronous test
%%
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

% Connect to 127.0.0.1，Namely V-REP software, the port number is 19997
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
% capture execution error caused by the code between try-catch, 
% once detected, stop and delete the connection 
disp('Program started');

traj=[];

if (clientID>-1)
    disp('Connected')
      
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    %Handle
    [~,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [~,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);

    %Command
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.5,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.5,vrep.simx_opmode_blocking);
    [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);
        
    for i=1:5000
        
        % Sensor Reading
       [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_buffer);
       
%        	[number returnCode,array position]=simxGetObjectPosition(number clientID,number objectHandle,number relativeToObjectHandle,number operationMode)
       [returnCode,position]=vrep.simxGetObjectPosition(clientID,camera,-1,vrep.simx_opmode_streaming  ); % -1 for absolute position
       [returnCode,angle]=vrep.simxGetObjectOrientation(clientID,camera,-1,vrep.simx_opmode_streaming  ); % -1 for absolute position
        
       if ~isempty(image)
        X=double([position angle*180/pi]);
        traj=[traj;X];
        figure(1)
        subplot(2,1,1); plot(traj(:,1),traj(:,2),'-b','LineWidth',2);
        axis([-5 5 -5 5])
        subplot(2,1,2); imshow(image);
       end
        pause(0.1)
    end
    
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
    
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
        
end

vrep.delete();
    