close all; clear; clc

addpath('../api')

vrep = remApi('remoteApi');
vrep.simxFinish(-1);

% 맵 설정
mapSize = 15; % 맵 크기
mapResolution = 0.1; % 맵 해상도

% Connect to V-REP
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
disp('Program started');

traj = [];
dd = [];

if (clientID > -1)
    disp('Connected')

    % 맵 초기화
    map = zeros(mapSize/mapResolution);
    
    % Start simulation
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
    
    % Get handles for robot and ref_point
    [~, robot] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking);
    [~, refPoint] = vrep.simxGetObjectHandle(clientID, 'ref_point', vrep.simx_opmode_blocking);
    % [~, lidar] = vrep.simxGetObjectHandle(clientID, 'fastHokuyo', vrep.simx_opmode_blocking);
    % 라이다 스크립트의 handle 가져오기
    [~, lidar1] = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor1', vrep.simx_opmode_blocking);
    [~, lidar2] = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor2', vrep.simx_opmode_blocking);
    
    % 측정된 데이터 받아오기
    [~, measuredData1] = vrep.simxGetStringSignal(clientID, 'measuredDataAtThisTime1', vrep.simx_opmode_blocking);
    [~, measuredData2] = vrep.simxGetStringSignal(clientID, 'measuredDataAtThisTime2', vrep.simx_opmode_blocking);
    
    % 측정된 데이터 언팩
    measuredData1 = vrep.simxUnpackFloats(measuredData1);
    measuredData2 = vrep.simxUnpackFloats(measuredData2);

    % Set robot's target velocity
    targetVelocity = 0.5; % Adjust the velocity as needed
    [~] = vrep.simxSetJointTargetVelocity(clientID, robot, targetVelocity, vrep.simx_opmode_blocking);
    
    figure;
    while true
        % Get ref_point position
        [~, refPointPosition] = vrep.simxGetObjectPosition(clientID, refPoint, -1, vrep.simx_opmode_streaming);
        
        if ~isempty(refPointPosition)
            % Get robot's current position
            [~, robotPosition] = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming);
            
            % Calculate target position (x, y coordinates only)
            targetPosition = [refPointPosition(1), refPointPosition(2), robotPosition(3)];
            
            % Move robot towards target position
            [~] = vrep.simxSetObjectPosition(clientID, robot, -1, targetPosition, vrep.simx_opmode_streaming);
            
            % Calculate target orientation (to maintain robot's direction)
            targetOrientation = atan2(refPointPosition(2) - robotPosition(2), refPointPosition(1) - robotPosition(1));
            
            % Set robot's target orientation
            [~] = vrep.simxSetObjectOrientation(clientID, robot, -1, [0, 0, targetOrientation], vrep.simx_opmode_streaming);
            
            % Store trajectory data
            traj = [traj; targetPosition(1:2)];
            
            % Update ref_point position
            [~] = vrep.simxGetObjectPosition(clientID, refPoint, -1, vrep.simx_opmode_streaming);

            % 로봇의 x, y 좌표 계산
            robotX = round((robotPosition(1) + mapSize/2) / mapResolution);
            robotY = round((robotPosition(2) + mapSize/2) / mapResolution);
            
            % 라이다 데이터 가져오기
            [~, detectionState, detectedPoint, ~, ~] = vrep.simxReadProximitySensor(clientID, lidar1, vrep.simx_opmode_blocking);
            % pause(0.1)
            d = detectionState;
            dd = [dd d];

            % 라이다로 감지된 점들을 맵에 표시
        
            % 점들을 맵 좌표로 변환하여 표시
            for i = 1:numel(measuredData1)/3
                x = round((robotPosition(1) + measuredData1(1 + (i-1)*3) + mapSize/2) / mapResolution);
                y = round((robotPosition(2) + measuredData1(2 + (i-1)*3) + mapSize/2) / mapResolution);
                map(y, x) = 1; % 장애물로 표시
            end
            
            % 맵 시각화
            clf
            hold on
            imagesc(flipud(map), 'XData', [-mapSize/2 mapSize/2], 'YData', [-mapSize/2 mapSize/2]);
            colormap(flipud(gray));
            plot(robotPosition(1), robotPosition(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            hold off
            axis equal
            xlim([-mapSize/2 mapSize/2]);
            ylim([-mapSize/2 mapSize/2]);
            drawnow
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