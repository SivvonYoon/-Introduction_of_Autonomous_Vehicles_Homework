close all; clear; clc
addpath('../api')
% simRemoteApi.start(19999): simple test
%simRemoteApi.start(19999,1300,false,true) ;
%%

i=1;
tf=10;
dt=0.05;
t=0.05:0.05:tf;

kp=0.5;
kw=5.0;

Puncte=[];
pp=[];
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
    [~,right_Motor]=vrep.siamxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    
    % kinect 
    [~,kinect_depth] = vrep.simxGetObjectHandle(clientID,'kinect_depth',vrep.simx_opmode_oneshot_wait);
    [~, kinect_rgb] = vrep.simxGetObjectHandle(clientID, 'kinect_rgb',vrep.simx_opmode_oneshot_wait);

    % The angles for kinect
    delta_z=57*pi/180;
    np_z=640;
    delta_x=43*pi/180;
    np_x=480;
    d_delta_z=-delta_z/2:delta_z/(np_z-1):delta_z/2;
    d_delta_x=-delta_x/2:delta_x/(np_x-1):delta_x/2;
     Ry=[cos(-pi),0,sin(-pi);0,1,0;-sin(-pi),0,cos(-pi)];

    [~,pos_ini]=(vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_blocking  )); % -1 for absolute position
    pos_ini=double(pos_ini);
    pre_pos=pos_ini;
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


   [~,posc]=vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_streaming  ); % -1 for absolute position
   [~,velc]=vrep.simxGetObjectVelocity(clientID,pioneer,vrep.simx_opmode_streaming  ); % -1 for absolute position
   posc=double(posc);
   velc=double(velc);
   [~,angle]=vrep.simxGetObjectOrientation(clientID,pioneer,-1,vrep.simx_opmode_streaming  ); % -1 for absolute position
   angc=double(angle);
   data(:,i)=[posc(1); posc(2); velc(1);velc(2); angc(3)];

    
    [~, res, depth] = vrep.simxGetVisionSensorDepthBuffer2(clientID,kinect_depth, vrep.simx_opmode_oneshot_wait); 
    
    [~, res, img1] = vrep.simxGetVisionSensorImage2(0,kinect_rgb, 0, vrep.simx_opmode_oneshot_wait);
    ximg1 = double(img1)/256;
    %imwrite(img1,'clown.png')
    
        for j=1:5:np_x%
            for k=1:5:np_z%
                y=depth(j,k)*3;
                if y<=5
                    x=y*tan(d_delta_x(1,j));
                    z=y*tan(d_delta_z(1,k));
                    punct=[-y;-z;x];
                    Puncte=[Puncte,punct];
                end
            end
        end
    Puncte=Ry*Puncte;

    ptCloud=pointCloud(Puncte');
    gridStep = 0.1;
    ptCloudA = % your code here
    
    rotationAngles = angc*180/pi;
    translation = posc-pos_ini;
    tform = % your code here


    ptCloudOut = % your code here
    if i>10

    pp=[pp;ptCloudOut.Location ];
    ppCloud=pointCloud(pp);

        figure(1)
    subplot(1,2,1);    imshow(img1);
    subplot(1,2,2);    plot3(pp(:,1),pp(:,2),pp(:,3),'.','Color','b')
    xlabel('x'); ylabel('y'); zlabel('z');grid on;


    end

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
        Puncte=[];
        pre_pos=posc;
        posc=[];
    end
    
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
        
end

vrep.delete();

%%

figure(2)
subplot(3,2,1); plot(t,xd,'-r',time,data(1,:),'--b','Linewidth',1.5)
                ylabel('x (m)','Interpreter','latex')
%                 axis([0 tf 0 5])
subplot(3,2,3); plot(t,yd,'-r',time,data(2,:),'--b','Linewidth',1.5)
                ylabel('y (m)','Interpreter','latex')
%                 axis([0 tf -1 1])
subplot(3,2,2); plot(t,vxd,'-r',time,data(3,:),'--b','Linewidth',1.5)
                ylabel('$\dot{x}$ (m)','Interpreter','latex')
%                 axis([0 tf -1.0 1.0])
subplot(3,2,4); plot(t,vyd,'-r',time,data(4,:),'--b','Linewidth',1.5)
                ylabel('$\dot{y}$ (m)','Interpreter','latex')
%                 axis([0 tf -1.0 1.0])
subplot(3,2,[5,6]); plot(t,thed,'-r',time,data(5,:),'--b','Linewidth',1.5)
                ylabel('\theta (rad)','Interpreter','latex')
%                 axis([0 tf -1 1])

figure(3)
pcshow(ppCloud);
    