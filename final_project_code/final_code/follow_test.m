close all; clear; clc

addpath('../api')

vrep = remApi('remoteApi');
vrep.simxFinish(-1);

% Connect to V-REP
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
disp('Program started');

traj = [];

if (clientID > -1)
    disp('Connected')
    
    % Start simulation
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
    
    % Get handles for robot and ref_point
    [~, robot] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking);
    [~, refPoint] = vrep.simxGetObjectHandle(clientID, 'ref_point', vrep.simx_opmode_blocking);
    
    % Set robot's target velocity
    targetVelocity = 0.5; % Adjust the velocity as needed
    [~] = vrep.simxSetJointTargetVelocity(clientID, robot, targetVelocity, vrep.simx_opmode_blocking);
    
    for i = 1:5000
        % Get ref_point position
        [~, refPointPosition] = vrep.simxGetObjectPosition(clientID, refPoint, -1, vrep.simx_opmode_streaming);
        
        if ~isempty(refPointPosition)
            % Move robot towards ref_point
            targetPosition = [refPointPosition(1:2), 0]; % Append a zero value for the z-axis
            [~] = vrep.simxSetObjectPosition(clientID, robot, -1, targetPosition, vrep.simx_opmode_streaming);
            
            % Store trajectory data
            traj = [traj; targetPosition(1:2)];
            
            % Visualization
            figure(1)
            plot(traj(:, 1), traj(:, 2), '-b', 'LineWidth', 2);
            axis([-5 5 -5 5])
            
            % Update ref_point position
            [~] = vrep.simxGetObjectPosition(clientID, refPoint, -1, vrep.simx_opmode_streaming);
        end
        
        pause(0.1)
    end
    
    % Stop robot's movement
    [~] = vrep.simxSetJointTargetVelocity(clientID, robot, 0, vrep.simx_opmode_blocking);
    
    % Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking);
    
    % Close connection to V-REP
    vrep.simxFinish(clientID);
end

vrep.delete();