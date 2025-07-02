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
        
        % Store trajectory data
        traj = [traj; depthImage];
        
        % Save depth image as .mat file
        save('depth_image.mat', 'depthImage');
    end
        
    pause(0.1)
    
    
    % Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking);
    
    % Close connection to V-REP
    vrep.simxFinish(clientID);
end

vrep.delete();

% Visualize depth image
figure(1);
imagesc(depthImage);
colormap gray;
colorbar;
axis image;
hold on;

map = occupancyMap(depthImage,10);
sv.Map = map;
sv.ValidationDistance = 0.01;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

planner = plannerRRTStar(ss,sv, ...
          ContinueAfterGoalReached=true, ...
          MaxIterations=2500, ...
          MaxConnectionDistance=0.3);

start = [0.5 0.5 0];
goal = [2.5 0.2 0];

rng(100,'twister') % repeatable result
[pthObj,solnInfo] = plan(planner,start,goal);

map.show
hold on
% Tree expansion
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
% Draw path
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)