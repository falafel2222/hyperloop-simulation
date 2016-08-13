% Hyperloop pod simulation for the OpenLoop Team
% Created by Alex Goldstein and Adam Shaw

% function [] = HyperloopSim()
    disp('Simulation Started')
    
        
    globals = globalData();
    pod = podData();
    tube = tubeData();
    
    failed=false;
    sensorType=[];
    sensorNum=[];
    sensorFailState=[];
    
    if globals.correctCovariance
         globals.peTopSIMCovConst = globals.peTopSIMCovConst;
         globals.peLeftSIMCovConst = globals.peLeftSIMCovConst;
         globals.peRightSIMCovConst = globals.peRightSIMCovConst;
         globals.pitotSIMCovConst = globals.pitotSIMCovConst;
         globals.distDownSIMCovConst = globals.distDownSIMCovConst;
         globals.distDownRailSIMCovConst = globals.distDownRailSIMCovConst;
         globals.distSideSIMCovConst = globals.distSideSIMCovConst;
         globals.IMUAccelSIMCovConst = globals.IMUAccelSIMCovConst;
         globals.IMUGyroSIMCovConst = globals.IMUGyroSIMCovConst;
        
         globals.peTopSIMCovLin = globals.peTopSIMCovLin;
         globals.peLeftSIMCovLin = globals.peLeftSIMCovLin;
         globals.peRightSIMCovLin = globals.peRightSIMCovLin;
         globals.pitotSIMCovLin = globals.pitotSIMCovLin;
         globals.distDownSIMCovLin = globals.distDownSIMCovLin;
         globals.distDownRailSIMCovLin = globals.distDownRailSIMCovLin;
         globals.distSideSIMCovLin = globals.distSideSIMCovLin;
         globals.IMUAccelSIMCovLin = globals.IMUAccelSIMCovLin;
         globals.IMUGyroSIMCovLin = globals.IMUGyroSIMCovLin;
               
         globals.peTopSIMCovZero = globals.peTopSIMCovZero;
         globals.peLeftSIMCovZero = globals.peLeftSIMCovZero;
         globals.peRightSIMCovZero = globals.peRightSIMCovZero;
         globals.pitotSIMCovZero = globals.pitotSIMCovZero;
         globals.distDownSIMCovZero = globals.distDownSIMCovZero;
         globals.distDownRailSIMCovZero = globals.distDownRailSIMCovZero;
         globals.distSideSIMCovZero = globals.distSideSIMCovZero;
         globals.IMUAccelSIMCovZero = globals.IMUAccelSIMCovZero;
         globals.IMUGyroSIMCovZero = globals.IMUGyroSIMCovZero;
    end
    
    kalmanFreq = globals.kalmanTimestep/globals.timestep;

    % temporary vectors for debugging purposes
    rolls = zeros(1,globals.numSteps/kalmanFreq+1);
    scannerDistances = zeros(globals.numSteps/kalmanFreq+1);

    
    noisyKalmanPosIMU = zeros(3,globals.numSteps/kalmanFreq+1);
    noisyKalmanVelIMU = zeros(3,globals.numSteps/kalmanFreq+1);
    noisyKalmanPosIMU(3,1) = .003;
    noisyKalmanRotIMU=[1 0 0; 0 1 0; 0 0 1];
    noisyKalmanQIMU=zeros(4,globals.numSteps/kalmanFreq+1);
    noisyKalmanQIMU(:,1)=[0.000001;0.000001;0.000001; sqrt(1-3*.000001^2)];
    % pod state variables
    transPos = zeros(3, globals.numSteps);
    transVel = zeros(3, globals.numSteps);
    transAcc = zeros(3, globals.numSteps);
    transPos(3,1) = .003;

    rotPos = zeros(3, globals.numSteps);
    rotVel = zeros(3, globals.numSteps);
    rotAcc = zeros(3, globals.numSteps);

    q=zeros(4, globals.numSteps);
    q(:,1)=[0.000001;0.000001;0.000001; sqrt(1-3*.000001^2)];

    
    %%%% For Kalman Filter %%%%    
    covariance = .001*eye(10);    
    % state is a 10x1 matrix of [position velocity quaternions]
    state = [transPos(:,1)' transVel(:,1)' q(:,1)']';
    kalmanHistory = zeros(10,globals.numSteps/kalmanFreq);
    kalmanHistory(:,1)=state;
    
    
    eBrakesActuated = false;    
    
    disp('Simulation Initialized')
    %%%BEGIN LOOPING THROUGH TIMESTEPS%%%
    for n = 2:globals.numSteps
%         if mod(n,1/(10*globals.timestep)) == 0
%             disp('--------------------------')
%             disp(n*globals.timestep)
%             disp(transPos(:,n-1)')
%             disp(transAcc(:,n-1)')
%         end
        
        %calculate rotation matrix
        q0=q(4,n-1);
        q1=q(1,n-1);
        q2=q(2,n-1);
        q3=q(3,n-1);
        rotMatrix=[1-2*q2^2-2*q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
                   2*(q1*q2+q0*q3) 1-2*q1^2-2*q3^2 2*(q2*q3-q0*q1);...
                   2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*q1^2-2*q2^2];

               
            %get all forces in local
            localForces=zeros(3,25);
            localPoints=zeros(3,25);
            forceIndex = 1;

               %%%%% AIR SKATES %%%%%

               % for each air skate, calculate the force
               for i = 1:length(pod.airskate(:,1,1))
                   for j = 1:length(pod.airskate(i,:,:))
                      point= rotMatrix*squeeze(pod.airskate(i,j,:)) + transPos(:,n-1); 
                      vertDist = tube.railHeight + point(3);
                      pointForce=SkateForce(vertDist,50e3,pod.skateSegmentLength);
                        localForces(3,forceIndex)= pointForce/length(pod.airskate(i,:,:));
                        localPoints(:,forceIndex) = squeeze(pod.airskate(i,j,:));
                        forceIndex = forceIndex + 1;

                   end
               end


                %%%%% SPACEX PUSHER %%%%%
                if transPos(1,n-1) < globals.pusherDistance
                    localPusherForce = rotMatrix\[globals.pusherForce; 0; 0];
                    localPusherPoint = pod.COM;

                    localForces(:,forceIndex) = localPusherForce;
                    localPoints(:,forceIndex) = localPusherPoint;
                    forceIndex = forceIndex + 1;
                end


                %%%%% DRAG FORCE %%%%%
                drag = pod.dragCoef*globals.airDensity*pod.height*pod.width/2*(transVel(1,n-1))^2;
                localDragForce = rotMatrix\[-drag;0;0];
                localDragPoint= [pod.length/2; 0; 0];

                localForces(:,forceIndex) = localDragForce;
                localPoints(:,forceIndex) = localDragPoint;
                forceIndex = forceIndex + 1;

                %%%GRAVITY FORCE%%
                gravityForce=[0 0 -1*globals.gravity]* pod.mass;
                localGravityForce=rotMatrix\gravityForce';
                localGravityPoint=pod.COM;

                localForces(:,forceIndex) = localGravityForce;
                localPoints(:,forceIndex) = localGravityPoint;
                forceIndex = forceIndex + 1;


            if globals.randomNoise

                forceSize=size(localForces);
                noise=zeros(3,forceSize(2));
                for i=1:forceSize(2)
                   magForce=norm(localForces(:,i));
                   noise(:,i)=(-1*globals.noiseModifier+2*globals.noiseModifier*rand())*magForce;
                end
                localForces=localForces+noise; 
            end
        
            

        
        %calculate torques in local
        netTorque=[0 0 0];
           for i=1:length(localPoints)
              torque=cross(localPoints(:,i)-pod.COM,localForces(:,i));
              netTorque=netTorque+torque';
           end
                   
%         display(netTorque)

        if (n <= 100)
           netTorque = [0 0 0];            
        end
        
        %get theta accel by tensor\torque
        rotAcc(:,n) = pod.tensor \ transpose(netTorque);
        
        %update omega, calculate omega matrix, update quaternions
        rotVel(:,n) = rotVel(:,n-1) + globals.timestep * rotAcc(:,n);

        OmegaMatrix=[0 rotVel(3,n) -rotVel(2,n) rotVel(1,n);...
                    -rotVel(3,n) 0 rotVel(1,n) rotVel(2,n);...
                     rotVel(2,n) -rotVel(1,n) 0 rotVel(3,n);...
                    -rotVel(1,n) -rotVel(2,n) -rotVel(3,n) 0;];

        qdot = 0.5*OmegaMatrix*q(:,n-1);
                
        qn=q(:,n-1)+globals.timestep*qdot;
        normQuat=sqrt(sum(qn.^2));
        qn=qn./normQuat;
        
        
        q(:,n) = qn;

        %calculate roll, pitch, and yaw
        rotPos(:,n-1) = [atan2(2*(q0*q1+q2*q3),1-2*(q1^2 + q2^2)); ...
                         asin(2*(q0*q2 - q3*q1));...
                         atan2(2*(q0*q3+q1*q2),1-2*(q2^2 + q3^2))];

        %get forces in global
        globalForces=rotMatrix*localForces;
        
        netForce=[0 0 0];
        for force=globalForces
            netForce = netForce+force';
        end
        
        if eBrakesActuated
            netForce = netForce + [-pod.eBrakeForce; 0; 0];
        end
        
        if (n < 100)
           netForce = [0 0 0];            
        end
        
        %get accel, velocity, position
        transAcc(:,n)=netForce/pod.mass;
        transVel(:,n) = transVel(:,n-1) + transAcc(:,n)*globals.timestep;
        transPos(:,n) = transPos(:,n-1) + transVel(:,n)*globals.timestep;
        
        %%%%%%%%%%% KALMAN FILTER STEP %%%%%%%%%%%%%%%%%%%%
        
        if mod(n,kalmanFreq) == 0
            
            roll = atan2(2*(q0*q1+ q2*q3),1-2*(q1^2 + q2^2));
            pitch = asin(2*(q0*q2 - q3*q1));
            rolls(n/kalmanFreq) = roll;
            % distance sensors
            distance1Pos = transPos(:,n)*ones(1,6)+rotMatrix*pod.bottomDistancePositions;
            for i=1:6
                [~, scanner1dist(i)] = DistanceFinder(distance1Pos(:,i)); 
            end

            distance2Pos = transPos(:,n)*ones(1,5)+rotMatrix*pod.downRailDistancePositions;
            for i=1:5
                [scanner2dist(i)] = TrackDistanceFinder(distance2Pos(:,i)); 
            end

            distance3Pos = transPos(:,n)*ones(1,5)+rotMatrix*pod.sideDistancePositions;
            for i=1:5
                [~, scanner3dist(i)] = InsideDistance(distance3Pos(:,i)); 
            end
            
            pitotRead=0.5*globals.airDensity*(dot(transVel(:,n),rotMatrix*pod.pitotDirection))^2;
            

            
            sensorData = [[scanner1dist']...
                [scanner2dist'; NaN;]...
                [scanner3dist'; NaN;]...
                [pitotRead;nan(5,1)]...
                [zeros(3,1); nan(3,1)]...
                [zeros(3,1); nan(3,1)]...
                [zeros(3,1); nan(3,1)]];
            execution =  [0 0 0 0 0 0 0];

            if mod(n,kalmanFreq*5) == 0
                execution(1) = 1;
                execution(2) = 1;
                execution(3) = 1;
            end
            
            if mod(n,kalmanFreq*8) == 0
                execution(4) = 1;
            end   
            
            IMUData = [transAcc(:,n)' rotVel(:,n)']';
%             disp('imu')
%             disp(IMUData)
            % add random noise

            IMUData = IMUData +...
                [sqrt(globals.IMUAccelSIMCovConst+globals.IMUAccelSIMCovLin.*abs(IMUData(1:3)-globals.IMUAccelSIMCovZero)).*randn(3,1);...
                sqrt(globals.IMUGyroSIMCovConst+globals.IMUGyroSIMCovLin.*abs(IMUData(4:6)-globals.IMUGyroSIMCovZero).*randn(3,1));];
%             
            sensorData = sensorData + [sqrt(globals.distDownSIMCovConst+globals.distDownSIMCovLin.*abs(scanner1dist'-globals.distDownSIMCovZero)).*randn(6,1)...
                [sqrt(globals.distDownRailSIMCovConst+globals.distDownRailSIMCovLin.*abs(scanner2dist'-globals.distDownRailSIMCovZero));NaN].*randn(6,1)...
                [sqrt(globals.distSideSIMCovConst+globals.distSideSIMCovLin.*abs(scanner3dist'-globals.distSideSIMCovZero));NaN].*randn(6,1)...
                [sqrt(globals.pitotSIMCovConst+globals.pitotSIMCovLin.*abs(pitotRead-globals.pitotSIMCovZero)); nan(5,1)].*randn(6,1)...
                [zeros(3,1); nan(3,1)]...
                [zeros(3,1); nan(3,1)]...
                 [zeros(3,1); nan(3,1)]];
            
%             if (n/kalmanFreq > 1)
            IMUData = IMUData + [rotMatrix\[0; 0; globals.gravity]; 0;0 ;0;];
            omegaX=IMUData(4);   %angular velocity in x from IMU
            omegaY=IMUData(5);   %angular velocity in y from IMU
            omegaZ=IMUData(6);
            
            q1=noisyKalmanQIMU(1,n/kalmanFreq);  
            q2=noisyKalmanQIMU(2,n/kalmanFreq);  
            q3=noisyKalmanQIMU(3,n/kalmanFreq);  
            q0=noisyKalmanQIMU(4,n/kalmanFreq);  
            noisyKalmanRotIMU=[1-2*q2^2-2*q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
                                 2*(q1*q2+q0*q3) 1-2*q1^2-2*q3^2 2*(q2*q3-q0*q1);...
                                 2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*q1^2-2*q2^2;];
            noisyKalmanOmegaIMU=[0 omegaZ -omegaY omegaX;...
                                -omegaZ 0 omegaX omegaY;...
                                omegaY -omegaX 0 omegaZ;...
                                -omegaX -omegaY -omegaZ 0;];
            
            noisyKalmanVelIMU(:,n/kalmanFreq+1) = noisyKalmanVelIMU(:,n/kalmanFreq) + globals.kalmanTimestep *(noisyKalmanRotIMU*(IMUData(1:3))-[0;0;globals.gravity]);
            noisyKalmanPosIMU(:,n/kalmanFreq+1) = noisyKalmanPosIMU(:,n/kalmanFreq) + globals.kalmanTimestep*noisyKalmanVelIMU(:,n/kalmanFreq);

            noisyKalmanQIMU(:,n/kalmanFreq+1)=noisyKalmanQIMU(:,n/kalmanFreq)+0.5*globals.kalmanTimestep*noisyKalmanOmegaIMU*noisyKalmanQIMU(:,n/kalmanFreq);
            
            normQuat=sqrt(sum((noisyKalmanQIMU(:,n/kalmanFreq+1)).^2));
            noisyKalmanQIMU(:,n/kalmanFreq+1)=noisyKalmanQIMU(:,n/kalmanFreq+1)./normQuat;
            %             else
%                 noisyKalmanVelIMU(:,n/kalmanFreq) = globals.kalmanTimestep *(IMUData(1:3));
%             end
            
%                 %disp('vel');
%                 disp(noisyKalmanVelIMU(:,n/kalmanFreq));
%                 %disp('pos');
%                 disp(noisyKalmanPosIMU(:,n/kalmanFreq));                
%                 scannerDistances(n/kalmanFreq) = scanner1dist;
%             disp(scanner1dist);
            if globals.sensorFailure
                if rand(1)<globals.failureRate
                    failed=true;
                    sensorType(end+1)=ceil(7*rand(1));
                    sensorList=sensorData(:,sensorType(end));
                    sensorNum(end+1)=ceil(rand(1)*length(find(~isnan(sensorList))));
                    if rand(1)>0.5
                        sensorFailState(end+1)=1;
                        sensorData(sensorNum(end),sensorType(end))=globals.sensorMaxs(sensorNum(end),sensorType(end));
                        fprintf('*****Sensor %d of type %d has failed, reading high.********\n',sensorNum(end),sensorType(end))
                    else
                        sensorFailState(end+1)=0;
                        sensorData(sensorNum(end),sensorType(end))=globals.sensorMins(sensorNum(end),sensorType(end));
                        fprintf('*****Sensor %d of type %d has failed, reading low.********\n',sensorNum(end),sensorType(end))
                    end
                end
                if globals.failurePersist && failed
                    for i=1:length(sensorType)
                       if sensorFailState(i)==1
                           sensorData(sensorNum(i),sensorType(i))=globals.sensorMaxs(sensorNum(i),sensorType(i));
                       elseif sensorFailState(i)==0
                           sensorData(sensorNum(i),sensorType(i))=globals.sensorMins(sensorNum(i),sensorType(i));
                       else
                           fprintf('**********************SOMETHING WRONG**********************\n')
                       end
                    end
                end
            end

            [sensorUse,numberUsed] = sensorFailureDetection(sensorData,globals,pod,tube);
            
            [state, covariance] = KalmanFilterHyperloop(state, covariance, IMUData, sensorData, execution, sensorUse,numberUsed, n/kalmanFreq,globals,pod,tube);
%              if mod(n,kalmanFreq*5) == 0
%                 distance1Pos
%                 transPos(:,n)
%                 state
%                 noisyKalmanPosIMU
%                 noisyKalmanPosIMU+pod.height/2
%             end
            
            kalmanHistory(:,n/kalmanFreq+1) = state;
%             disp('---------------------')
%             disp(n/kalmanFreq)
%             disp(kalmanHistory(3,n/kalmanFreq))
%             disp(noisyKalmanPosIMU(3,n/kalmanFreq))
%             disp(transPos(3,n))
        end
        
        
        %check collisions
        endSimul=false;
        for point = pod.collisionPoints;
            realPoint = rotMatrix*point + transPos(:,n);
            [distance, ~]=DistanceFinder(realPoint);
            if mod(n,kalmanFreq) == 0
                kalmanPoint = rotMatrix*point + kalmanHistory(1:3,n/kalmanFreq);
                [kalmanDistance, ~] = DistanceFinder(kalmanPoint);
                if kalmanDistance <= 0
%                    disp(n); 
                end
            end

            if distance<=0
                norm(point)
                disp('Collision Occurred at timestep');
                disp(n);
                endSimul=true;
                break
            end
        end
        if endSimul
           break 
        end

                
    end
    
    display(size(kalmanHistory(3,:)))
    display(size(transPos(3,:)))
    figure;
    subplot(3,1,1)
    plot(kalmanHistory(1,:))
    hold all
    plot(downsample(transPos(1,:),kalmanFreq),':');
    plot(noisyKalmanPosIMU(1,:),':');
    legend('Kalman','Physical','IMU')
    title('X-position')
    hold off
    subplot(3,1,2)
    plot(kalmanHistory(2,:))
    hold all
    plot(downsample(transPos(2,:),kalmanFreq),':');
    plot(noisyKalmanPosIMU(2,:),':');
    legend('Kalman','Physical','IMU')
    title('Y-position')
    hold off
    subplot(3,1,3)
    plot(kalmanHistory(3,:))
    hold all
    plot(downsample(transPos(3,:),kalmanFreq),':');
    plot(noisyKalmanPosIMU(3,:),':');
    legend('Kalman','Physical','IMU')
    title('Z-position')
    hold off
    figure;
    subplot(3,1,3)
    hold all
    IMUvReal=noisyKalmanVelIMU(3,1:end-1)-downsample(transVel(3,:),kalmanFreq);
    plot(kalmanHistory(6,:));
    plot(downsample(transVel(3,:),kalmanFreq),':');
    plot(noisyKalmanVelIMU(3,:),':');
    title('Z-velocity')
    subplot(3,1,1)
    hold all
%     IMUvReal=noisyKalmanVelIMU(3,1:end-1)-downsample(transVel(3,:),kalman
%     Freq);
    plot(kalmanHistory(4,:))
    plot(downsample(transVel(1,:),kalmanFreq),':');
    plot(noisyKalmanVelIMU(1,:),':');
    title('X-velocity')
    subplot(3,1,2)
    hold all
%     IMUvReal=noisyKalmanVelIMU(3,1:end-1)-downsample(transVel(3,:),kalmanFreq);

    plot(kalmanHistory(5,:))
    plot(downsample(transVel(2,:),kalmanFreq),':');
        plot(noisyKalmanVelIMU(2,:),':');
    title('Y-velocity')
    
    legend('Kalman','Physical','IMU')
    hold off
    

   


% end