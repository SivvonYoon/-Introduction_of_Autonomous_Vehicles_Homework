% V-REP과의 연결 설정
vrep = remApi('remoteApi');
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

% 맵 객체 핸들 얻기
[~, mapHandle] = vrep.simxGetObjectHandle(clientID, 'map', vrep.simx_opmode_oneshot_wait);

% 맵 크기 및 위치 정보 가져오기
[~, mapPosition] = vrep.simxGetObjectPosition(clientID, mapHandle, -1, vrep.simx_opmode_oneshot_wait);
[~, mapSize] = vrep.simxGetObjectFloatParameter(clientID, mapHandle, 10, vrep.simx_opmode_oneshot_wait);

% 맵 정보 출력
fprintf('맵 위치: (%.2f, %.2f, %.2f)\n', mapPosition);
fprintf('맵 크기: %.2f\n', mapSize);

% 연결 종료
vrep.simxFinish(clientID);
vrep.delete();