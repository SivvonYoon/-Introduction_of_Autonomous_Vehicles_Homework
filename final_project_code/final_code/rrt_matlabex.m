close all; clear; clc
%% occupancy map

%load map image and crop the size
image = imread('proj.pgm');
imageCropped = image(1:560,1:430);
% imshow(imageCropped)

imageBW = imageCropped < 100;
% imshow(imageBW)


%Create an occupany map from an example map and set map resolution as 20 cells/meter.
map = binaryOccupancyMap(imageBW,20);
show(map)

%% 

%Create a occupancyMap-based state validator using the created state space.
ss = stateSpaceSE2;

sv = validatorOccupancyMap(ss); 
sv.Map = map;

% Set validation distance for the validator.
sv.ValidationDistance = 0.01; 


ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;


% Reduce max iterations and increase max connection distance.
planner.MaxIterations = 2500;
planner.MaxConnectionDistance = 5;

% Set the start and goal states.
start = [15, 5, 0];
goal = [5, 25, 0];

%Plan a path with default settings.
tic
rng(100, 'twister') % repeatable result
[pthObj, solnInfo] = plan(planner,start,goal);
toc

    interpolatedSmoothWaypoints = copy(pthObj);
    interpolate(interpolatedSmoothWaypoints,1000)
    
    waypoints = interpolatedSmoothWaypoints.States;

    
% visualize
map.show;
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path
plot(waypoints(:,1),waypoints(:,2),'b-.','LineWidth',2); % draw path
