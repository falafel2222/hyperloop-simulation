

    globals = globalData();
    pod = podData();
    tube = tubeData();
    

timestep=0.001;
File=importdata(fileName,'\t',1);
log=File.textdata;
window=1;
matdata=cellfun(@str2num,log(22:end,1:end-1));
xGyro=filter(ones(1,window)/window,1,matdata(:,1));
yGyro=filter(ones(1,window)/window,1,matdata(:,2));
zGyro=filter(ones(1,window)/window,1,matdata(:,3));
xAcc=filter(ones(1,window)/window,1,matdata(:,4)*9.81);
yAcc=filter(ones(1,window)/window,1,matdata(:,5)*9.81);
zAcc=filter(ones(1,window)/window,1,matdata(:,6)*9.81);

time=0:timestep:timestep*(nSteps-1);
fulltime=0:timestep:timestep*(length(xAcc)-1);

qSt=[0.000001;0.000001;0.000001; sqrt(1-3*.000001^2)];
    
grav=[xAStart,yAStart,zAStart]
gravityVal=norm(grav)
grav=grav/gravityVal;

axis=cross(grav,[0,0,1]);
axis=axis'/norm(axis);

angle=acos(dot(grav,[0,0,1])/(norm(grav)));
    

startCov=diag([0.0000001,0.0000001,0.0000001,0.0000001,0.0000001,0.0000001,0.00001,0.00001,0.00001,0.00001]);
startState=[0;0;0;0;0;0;qSt;];


    transPos = zeros(3, nSteps);
    transVel = zeros(3, nSteps);
    transAcc = zeros(3, nSteps);
    Var=zeros(10,nSteps);
    rotVel = zeros(3, nSteps);
    rotPos=zeros(3,nSteps);
    
    q=zeros(4, nSteps);
    
    q(:,1)=qSt;
    
    
        q0=q(4,1);
        q1=q(1,1);
        q2=q(2,1);
        q3=q(3,1);
        rotPos(:,1) = [atan2(2*(q0*q1+q2*q3),1-2*(q1^2 + q2^2)); ...
                         asin(2*(q0*q2 - q3*q1));...
                         atan2(2*(q0*q3+q1*q2),1-2*(q2^2 + q3^2))];

    sensorUse=[1:6;1:6;1:6;1:6;1:6;1:6;1:6]';
    numberUsed=[6,5,5,1,3,3,3];
    sensorData=zeros(6,7);
    execution=[0 0 0 0 0 0 0];
    
    state=startState;
    cov=startCov;
    for i=2:nSteps
        if round(i/1000)==i/1000
            disp(i)
        end
        transPos(:,i)=nextState(1:3);
        transVel(:,i)=nextState(4:6);
        transAcc(:,i-1)=IMUData(1:3);
        q(:,i)=nextState(7:10);
        
        q0=q(4,i);
        q1=q(1,i);
        q2=q(2,i);
        q3=q(3,i);
        rotPos(:,i) = [atan2(2*(q0*q1+q2*q3),1-2*(q1^2 + q2^2)); ...
                         asin(2*(q0*q2 - q3*q1));...
                         atan2(2*(q0*q3+q1*q2),1-2*(q2^2 + q3^2))];
        Var(:,i)=diag(nextCov);
        
        state=nextState;
        cov=nextCov;
    end
    
    figure();
    subplot(2,3,1);
    plot(fulltime,xGyro,'r');
    title('x rot vel vs. time');
    xlabel('seconds');
    ylabel('rad/s');

    subplot(2,3,2);
    plot(fulltime,yGyro,'b');
    title('y rot vel vs. time');
    xlabel('seconds');
    ylabel('rad/s');

    subplot(2,3,3);
    plot(fulltime,zGyro,'g');
    title('z rot vel vs. time');
    xlabel('seconds');
    ylabel('rad/s');

    subplot(2,3,4);
    plot(fulltime,xAcc,'r');
    title('x-axis accel vs. time');
    xlabel('seconds');
    ylabel('m/s^2');

    subplot(2,3,5);
    plot(fulltime,yAcc,'b');
    title('y-axis accel vs. time');
    xlabel('seconds');
    ylabel('m/s^2');

    subplot(2,3,6);
    plot(fulltime,zAcc,'g');
    title('z-axis accel vs. time');
    xlabel('seconds');
    ylabel('m/s^2');
    
    figure();
    subplot(3,3,1);
    plot(time,transVel(1,:),'r');
    title('x vel vs. time');
    xlabel('seconds');
    ylabel('m/s');

    subplot(3,3,2);
    plot(time,transVel(2,:),'b');
    title('y vel vs. time');
    xlabel('seconds');
    ylabel('m/s');

    subplot(3,3,3);
    plot(time,transVel(3,:),'g');
    title('z vel vs. time');
    xlabel('seconds');
    ylabel('m/s');

    subplot(3,3,4);
    plot(time,transPos(1,:),'r');
    title('x pos vs. time');
    xlabel('seconds');
    ylabel('m');

    subplot(3,3,5);
    plot(time,transPos(2,:),'b');
    title('y pos vs. time');
    xlabel('seconds');
    ylabel('m');

    subplot(3,3,6);
    plot(time,transPos(3,:),'g');
    title('z pos vs. time');
    xlabel('seconds');
    ylabel('m');

    subplot(3,3,7);
    plot(time,rotPos(1,:)*180/pi,'r');
    title('roll vs. time');
    xlabel('seconds');
    ylabel('deg');

    subplot(3,3,8);
    plot(time,rotPos(2,:)*180/pi,'b');
    title('pitch vs. time');
    xlabel('seconds');
    ylabel('deg');

    subplot(3,3,9);
    plot(time,rotPos(3,:)*180/pi,'g');
    title('yaw vs. time');
    xlabel('seconds');
    ylabel('deg');
    
    figure();
    plot3(transPos(1,:),transPos(2,:),transPos(3,:));
    title('Global Path')
    
        
        
        
