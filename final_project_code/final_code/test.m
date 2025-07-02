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
    % Lidar 객체 핸들 가져오기
    [returnCode, lidar] = vrep.simxGetObjectHandle(clientID, './sensor1', vrep.simx_opmode_blocking);

    % Lidar 데이터 저장 변수 초기화
    measuredData = [];
    
    % Set robot's target velocity
    targetVelocity = 0.5; % Adjust the velocity as needed
    [~] = vrep.simxSetJointTargetVelocity(clientID, robot, targetVelocity, vrep.simx_opmode_blocking);
    
    for i = 1:500
        % Get ref_point position
        [~, refPointPosition] = vrep.simxGetObjectPosition(clientID, refPoint, -1, vrep.simx_opmode_streaming);
        
        % Lidar 데이터 가져오기
        [returnCode, data] = vrep.simxGetStringSignal(clientID, 'measuredDataAtThisTime', vrep.simx_opmode_blocking);
        if returnCode == vrep.simx_return_ok
            strData = char(data');
            newData = sscanf(strData, '%f');
            measuredData = [measuredData; newData];  % 데이터 저장
        end
        
        % SLAM 맵 업데이트 함수 호출
        updateSLAMMap(measuredData);

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

% SLAM 맵 업데이트 함수
function updateSLAMMap(measuredData)
    % 맵 크기와 해상도 설정
    mapResolution = 0.1;  % 맵 해상도 (미터/셀)
    mapSize = [10, 10] / mapResolution;  % 맵 크기 (셀)

    % 맵 생성
    map = robotics.OccupancyGrid(mapSize(1), mapSize(2), mapResolution);

    % Lidar 데이터를 이용하여 맵 업데이트
    robotPose = [0, 0];  % 로봇의 위치
    for i = 1:size(measuredData, 1)
        scan = measuredData(i, :);
        angles = linspace(-pi/2, pi/2, numel(scan));
        ranges = scan';
        
        % 레이저 빔의 시작점과 끝점 계산
        [x, y] = pol2cart(angles, ranges);
        endpts = [x + robotPose(1), y + robotPose(2)];
        
        % 맵 업데이트 로직 추가
        insertRay(map, robotPose, endpts);
    end
    
    % 맵 시각화
    show(map);
end