% V-REP와 연결
addpath('C:\Users\jaewan\Desktop\자율주행시스템개론\SLAM_project\api')
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 1300, 5);
disp('Program started');

if (clientID > -1)
    disp('Connected')

    % 시작
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);

    % Lidar 객체 핸들 가져오기
    [returnCode, lidar] = vrep.simxGetObjectHandle(clientID, './sensor1', vrep.simx_opmode_blocking);
    % vrep에서 객체 가져오기
    [~, leftMotor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking);
    [~, rightMotor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking);
    [~, robot] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking);
    
    % Lidar 데이터 저장 변수 초기화
    measuredData = [];

    % 실행 시간 측정 시작
    tic;

    while toc < 90  % 90초 동안 실행
        % Lidar 데이터 가져오기
        [returnCode, data] = vrep.simxGetStringSignal(clientID, 'measuredDataAtThisTime', vrep.simx_opmode_blocking);
        if returnCode == vrep.simx_return_ok
            strData = char(data');
            newData = sscanf(strData, '%f');
            measuredData = [measuredData; newData];  % 데이터 저장
        end
        
        % SLAM 맵 업데이트 함수 호출
        updateSLAMMap(measuredData);
        
        % 주행 속도 조절 (필요에 따라 추가)
        pause(0.1);
    end

    % 시뮬레이션 종료
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking);

    % V-REP 연결 종료
    vrep.simxFinish(clientID);
    disp('Program ended');
else
    disp('Failed to connect to V-REP');
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