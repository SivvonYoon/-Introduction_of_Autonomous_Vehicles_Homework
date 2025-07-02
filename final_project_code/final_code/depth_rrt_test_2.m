close all; clear; clc

load depth_image_0.mat
map = occupancyMap(depthImage, 10);

start = [200 400 0];
goal = [400 200 0];

% Create state space
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

% Create validator with occupancy map
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.01;

% Create RRT* planner
planner = plannerRRTStar(ss, sv);
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 2500;
planner.MaxConnectionDistance = 0.3;

rng(100,'twister') % for reproducibility
[pthObj, solnInfo] = plan(planner, start, goal);

map.show;
hold on;
% Tree expansion
plot(solnInfo.TreeData(:, 1), solnInfo.TreeData(:, 2), '.-')
% Draw path
plot(pthObj.States(:, 1), pthObj.States(:, 2), 'r-', 'LineWidth', 2)