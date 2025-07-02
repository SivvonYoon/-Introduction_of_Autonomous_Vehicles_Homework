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
    
    % Get handle for Kinect sensor
    [~, kinect] = vrep.simxGetObjectHandle(clientID, 'kinect_depth', vrep.simx_opmode_blocking);

    % Get depth image from Kinect
    [returnCode, resolution, depthImage] = vrep.simxGetVisionSensorDepthBuffer2(clientID, kinect, vrep.simx_opmode_blocking);
    
    if returnCode == vrep.simx_return_ok
        % Process depth image
        depthImage = reshape(depthImage, resolution);
        
        % Normalize depth values from 0.4~0.5 to 0~1
        depthImage = (depthImage - 0.4) / (0.5 - 0.4);
        
        % Clamp values below 0 to 0 and above 1 to 1
        depthImage(depthImage <= 0.9) = 0;
        depthImage(depthImage >= 0.9) = 1;
        
        % Visualize depth image
        figure(1);
        imagesc(depthImage);
        colormap gray;
        colorbar;
        axis image;
        
        % Store trajectory data
        traj = [traj; depthImage];
    end
        
    pause(0.1)
    
    
    % Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking);
    
    % Close connection to V-REP
    vrep.simxFinish(clientID);
end

vrep.delete();






