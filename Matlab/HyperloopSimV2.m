% Hyperloop pod simulation for the OpenLoop Team
% Created by Alex Goldstein and Adam Shaw

function [] = HyperloopSimV2()
    disp('Simulation Started')
    
    globals = globalData();
    pod = podData();
    tube = tubeData();
    
    kalmanFreq = globals.kalmanTimestep/globals.timestep;
    
    %set initial variables
    % Tube conditions
    DISTANCETOFLAT=0.72136;
    DRAG_COEFFICIENT = 0.2;
    AIR_PRESSURE = 1.45;
    AIR_DENSITY = 6900*AIR_PRESSURE/(287*298);
    
    skateLength = pod.length; %m

    idealStartHeight=0.001;

    % pod state variables
    transPos = zeros(3, globals.numSteps);
    transVel = zeros(3, globals.numSteps);
    transAcc = zeros(3, globals.numSteps);
    transPos(3,1) = -1*(DISTANCETOFLAT-(pod.height/2)-idealStartHeight);
    transPos(3,1)

    rotPos = zeros(3, globals.numSteps);
    rotVel = zeros(3, globals.numSteps);
    rotAcc = zeros(3, globals.numSteps);

    q=zeros(4, globals.numSteps);
    q(:,1)=[0.000001;0.000001;0.000001; sqrt(1-3*.000001^2)];
               
                    
    %thrust generating points
    numSegments = 2;
    
    airSkateRight=zeros(3,numSegments);
    airSkateLeft=zeros(3,numSegments);
    for i=1:numSegments
       airSkateRight(:,i) = [pod.length/2-(i-1)*skateLength/(numSegments-1),-pod.width/2,-pod.width/2];
       airSkateLeft(:, i) = [pod.length/2-(i-1)*skateLength/(numSegments-1), pod.width/2,-pod.height/2];
    end    
    skatePoints=[airSkateLeft airSkateRight];                
       
    % points for the rail wheels, while not extended
    wheelGap = .04; %m
    wheelVert = .05; %m   
    numWheels = 8;
    
    rightRailWheelPoints = zeros(3,numWheels);
    leftRailWheelPoints = zeros(3,numWheels);
    for i=1:numWheels
       rightRailWheelPoints(:,i) = [pod.length/2-(i-.5)*pod.length/numWheels,...
           -wheelGap/2, wheelVert-pod.height/2];
       leftRailWheelPoints(:, i) = [pod.length/2-(i-.5)*pod.length/numWheels,...
            wheelGap/2, wheelVert-pod.height/2];
    end
    
    %%%% for kalman filter %%%%
    
    covariance = .001*eye(10);
    
    % state is a 10x1 matrix of [position velocity quaternions]
    state = [transPos(:,1)' transVel(:,1)' q(:,1)']';
    
    kalmanHistory = zeros(10,globals.numSteps/kalmanFreq);
    
    
    disp('Simulation Initialized')
    %%%BEGIN LOOPING THROUGH TIMESTEPS%%%
    for n = 2:globals.numSteps
        if mod(n,1000) == 0
            disp('--------------------------')
            disp(n*globals.timestep)
            disp(transPos(:,n-1)')
            disp(transAcc(:,n-1)')
        end
         
        %calculate rotation matrix
        q0=q(4,n-1);
        q1=q(1,n-1);
        q2=q(2,n-1);
        q3=q(3,n-1);
        rotMatrix=[1-2*q2^2-2*q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
                   2*(q1*q2+q0*q3) 1-2*q1^2-2*q3^2 2*(q2*q3-q0*q1);...
                   2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*q1^2-2*q2^2];
                
        %get all forces in local
        localForces=[];
        localPoints=[];
           %forces already in local

           %%%%% AIR SKATES %%%%%
           skateForces=zeros(3,length(skatePoints));
           for i=1:length(skatePoints)
                point= rotMatrix*skatePoints(:,i) + transPos(:,n-1);
                [~, vertDist]=DistanceFinder(point);
                
                pointForce=SkateForce(vertDist,11e3,skateLength);  
                skateForces(3,i)=pointForce/(2*length(airSkateRight));
            end
            localForces=[localForces skateForces];
            localPoints=[localPoints skatePoints];
            

            %%%%% SPACEX PUSHER %%%%%
            if transPos(1,n-1) < globals.pusherDistance
                localPusherForce = rotMatrix\[globals.pusherForce; 0; 0];
                localPusherPoint = [-pod.length/2;0;0];
                
                localForces=[localForces localPusherForce];
                localPoints=[localPoints localPusherPoint];
            end
            
            
            %%%%% DRAG FORCE %%%%%
            drag = DRAG_COEFFICIENT* AIR_DENSITY*pod.width*pod.height*(transVel(1,n-1))^2 / 2;
            localDragForce = rotMatrix\[-drag;0;0];
            localDragPoint= [pod.length/2; 0; 0];
            
            localForces=[localForces localDragForce];
            localPoints=[localPoints localDragPoint];
            
            %%%GRAVITY FORCE%%
            gravityForce=[0 0 -1*globals.gravity]* pod.mass;
            localGravityForce=rotMatrix\gravityForce';
            localGravityPoint=[0;0;0];

            localForces=[localForces localGravityForce];
            localPoints=[localPoints localGravityPoint];
           
        
        
        if globals.randomNoise
            
            forceSize=size(localForces);
            noise=zeros(3,forceSize(2));
            for i=1:forceSize(2)
               magForce=norm(localForces(:,i));
               noise(:,i)=-1*globals.noiseModifier+(2*globals.noiseModifier*rand())*magForce;
            end
            localForces=localForces+noise; 
        end
        
        %calculate torques in local
        netTorque=[0 0 0];
           for i=1:length(localPoints)
              torque=cross(localPoints(:,i),localForces(:,i));
              netTorque=netTorque+torque';
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
                
        %get accel, velocity, position
        transAcc(:,n)=netForce/pod.mass;
        transVel(:,n) = transVel(:,n-1) + transAcc(:,n)*globals.timestep;
        transPos(:,n) = transPos(:,n-1) + transVel(:,n)*globals.timestep;
        
        
        %%%%%%%%%%% KALMAN FILTER STEP %%%%%%%%%%%%%%%%%%%%
        
        % for bare bones implementation testing, we'll have no corrective
        % anything and perfect IMU data
        
        if mod(n,kalmanFreq) == 0
            
            %%%%% GET SENSOR DATA %%%%%
            
            %%%%% GET ROLL, PITCH, YAW %%%%%
            
            roll = atan2(2*(q0*q1+ q2*q3),1-2*(q1^2 + q2^2));
            pitch = asin(2*(q0*q2 - q3*q1));
            
            % laser scanners
            scanner1Pos = [0; 0; transPos(3,n)-pod.height/2];
            [~, scanner1dist] = DistanceFinder(rotMatrix*scanner1Pos);
            
            scanner2Pos = [0; 0; 0];
            [~, scanner2dist] = DistanceFinder(rotMatrix*scanner2Pos); 
            
            scanner3Pos = [0; 0; 0];
            [~, scanner3dist] = InsideDistance(rotMatrix*scanner3Pos);

            
            
            
            sensorData = [[scanner1dist; pitch] [0;0] [0;0] [0;0] [0;0] [0;0] [0;0]];
            execution =  [1 0 0 0 0 0 0];

            IMUData = [transAcc(:,n)' rotVel(:,n)']';
            IMUData = IMUData + [0 0 9.81 0 0 0]';

            % add random noise
            IMUData = IMUData + (.001*(rand(1)-.5)).*IMUData;
            sensorData = (1 + .01*(rand(1)-.5)).*sensorData;

            [state, covariance] = KalmanFilterHyperloop(state, covariance, IMUData, sensorData, execution);

            kalmanHistory(:,n/kalmanFreq) = state;
        end
        
        
        %check collisions
        endSimul=false;
%         for point = pod.collisionPoints;
%             point = rotMatrix*point + transPos(:,n);
%             [distance, ~]=DistanceFinder(point);
%             if distance<=0
%                 norm(point)
%                 disp('Collision Occurred at timestep');
%                 disp(n);
%                 endSimul=true;
%                 break
%             end
%         end
        if endSimul
           break 
        end

                
    end
    
    display(size(kalmanHistory(3,:)))
    display(size(transPos(3,:)))
    plot(kalmanHistory(3,:))
    hold on
    plot(downsample(transPos(3,:),kalmanFreq));
    hold off
    
end