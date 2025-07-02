close all; clear; clc
addpath('api')
% simRemoteApi.start(19999): simple test
% simRemoteApi.start(19999,1300,false,true) : synchronous test
%%

i=1;
tf=10;
dt=0.05;
t=0.05:0.05:tf;

kp=%;
kw=%;


%%
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    
    disp('Connected to remote API server');

     vrep.simxSynchronous(clientID,true);

     vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    %Handle
    [~,pioneer]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    [~,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [~,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    [~,pos_ini]=(vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_blocking  )); % -1 for absolute position
    pos_ini=double(pos_ini);
    [~,ang_ini]=(vrep.simxGetObjectOrientation(clientID,pioneer,-1,vrep.simx_opmode_blocking  )); % -1 for absolute position
    ang_ini=double(ang_ini);
    %Command
    [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.0,vrep.simx_opmode_blocking);
    xd=pos_ini(1)+0.5*t;
    vxd=0.5*ones(1,length(t));
    yd=pos_ini(2)+0.5*cos(2*pi*t/tf)-0.5;
    vyd=-0.5*2*pi/tf*sin(2*pi*t/tf);
    for i=1:length(t)
         thed(i)=atan2(vyd(i),vxd(i)); 
    end

    for i=1:length(t)
% for i=1:20
       [~,posc]=vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_streaming  ); % -1 for absolute position
       [~,velc]=vrep.simxGetObjectVelocity(clientID,pioneer,vrep.simx_opmode_streaming  ); % -1 for absolute position
       posc=double(posc); velc=double(velc); 
       [~,angle]=vrep.simxGetObjectOrientation(clientID,pioneer,-1,vrep.simx_opmode_streaming  ); % -1 for absolute position
       angc=double(angle);
      data(:,i)=[posc(1); posc(2); velc(1);velc(2); angc(3)];
      
      vd=% code here;
      omegad=%code here;
      d=0.06; %wheels separation
      r_w=0.0275; % wheel radius
      v_r=%code here;
      v_l=%code here;
      
      [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,v_l,vrep.simx_opmode_blocking);
      [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,v_r,vrep.simx_opmode_blocking);   
        
%        X=[position angle*180/pi];
%        disp(velc)
       
       vrep.simxSynchronousTrigger(clientID);
       time(i)=t(i);
    end
    
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
        
end

vrep.delete();

%%

figure(1)
subplot(3,2,1); plot(t,xd,'-r',time,data(1,:),'--b','Linewidth',1.5)
                ylabel('x (m)','Interpreter','latex')
                axis([0 tf 0 5])
subplot(3,2,3); plot(t,yd,'-r',time,data(2,:),'--b','Linewidth',1.5)
                ylabel('y (m)','Interpreter','latex')
                axis([0 tf -1 1])
subplot(3,2,2); plot(t,vxd,'-r',time,data(3,:),'--b','Linewidth',1.5)
                ylabel('$\dot{x}$ (m)','Interpreter','latex')
                axis([0 tf -1.0 1.0])
subplot(3,2,4); plot(t,vyd,'-r',time,data(4,:),'--b','Linewidth',1.5)
                ylabel('$\dot{y}$ (m)','Interpreter','latex')
                axis([0 tf -1.0 1.0])
subplot(3,2,[5,6]); plot(t,thed,'-r',time,data(5,:),'--b','Linewidth',1.5)
                ylabel('\theta (rad)','Interpreter','latex')
                axis([0 tf -1 1])
    