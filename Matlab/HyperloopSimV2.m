% Hyperloop pod simulation for the OpenLoop Team
% Created by Alex Goldstein and Adam Shaw

function [] = HyperloopSimV2()
    disp('Simulation Started')
    
    stripDistances=[100 200 300 400 500 600 700 800 900 1000 1100 1200 1300 1400 1500 1600 1700 1800 1900 2000 2100 2200 2300 2400 2500 2600 2700 2800 2900 3000 3100 3200 3300 3400 3500 3600 3700 3800 3900 4000 4100 4200 ...
        4280 4280+(1/3) 4280+(2/3) 4281 4281+(1/3) 4281+(2/3) 4282 4281+(1/3) 4281+(2/3) 4282 4282+(1/3) 4282+(2/3) 4283 4283+(1/3) 4283+(2/3) 4284 4284+(1/3) 4284+(2/3) 4285 4285+(1/3) ...
        4300 4400 4500 4600 4700 ...
        4780 4780+(1/3) 4780+(2/3) 4781 4781+(1/3) 4781+(2/3) 4782 4782+(1/3) 4782+(2/3) 4783 ...
        4800 4900 5000 5100 5200]*12*0.0254;
    stripWidth=2*0.0254;
    
    
    randomNoise=true;
    noiseModifier=0.0001;
    
    %set initial variables
    % Tube conditions
    DISTANCETOFLAT=0.72136;
    DRAG_COEFFICIENT = 0.2;
    AIR_PRESSURE = 1.45;
    AIR_DENSITY = 6900*AIR_PRESSURE/(287*298);
    PUSHER_FORCE = 17640; % newtons
    PUSHER_DISTANCE = 243; % meters
    
    %pod dimensions
    podHeight = 1; %m
    podWidth = 1; %m
    podLength = 5; %m
    skateLength = podLength; %m

    mass = 1750; %kg
    I = [1.0/12*mass*(podHeight^2 + podWidth^2) 0 0; ...
         0 1.0/12*mass*(podHeight^2 + podLength^2) 0; ...
         0 0 1.0/12*mass*(podLength^2 + podWidth^2)];
    CoM = [0,0,0];

    timestep = .0001; %sec
    maxTime = 1; %sec
    numSteps = maxTime / timestep;
    idealStartHeight=0.001;

    % pod state variables
    transPos = zeros(3, numSteps);
    transVel = zeros(3, numSteps);
    transAcc = zeros(3, numSteps);
    transPos(3,1) = -1*(DISTANCETOFLAT-(podHeight/2)-idealStartHeight);
    transPos(3,1)

    rotPos = zeros(3, numSteps);
    rotVel = zeros(3, numSteps);
    rotAcc = zeros(3, numSteps);

    q=zeros(4, numSteps);
    q(:,1)=[0;0;.0; 1];
       
        
    %collision points
    collisionPoints =   [podLength/2   podWidth/2  -podHeight/2-CoM(3); ...
                         podLength/2  -podWidth/2  -podHeight/2-CoM(3); ...
                        -podLength/2   podWidth/2  -podHeight/2-CoM(3); ...
                        -podLength/2  -podWidth/2  -podHeight/2-CoM(3); ...
                         podLength/2   podWidth/2   podHeight/2-CoM(3); ...
                         podLength/2  -podWidth/2   podHeight/2-CoM(3); ...
                        -podLength/2   podWidth/2   podHeight/2-CoM(3); ...
                        -podLength/2  -podWidth/2   podHeight/2-CoM(3)];
    
                    
    %thrust generating points
    numSegments = 2;
    
    airSkateRight=zeros(3,numSegments);
    airSkateLeft=zeros(3,numSegments);
    for i=1:numSegments
       airSkateRight(:,i) = [podLength/2-(i-1)*skateLength/(numSegments-1),-podWidth/2,-podHeight/2];
       airSkateLeft(:, i) = [podLength/2-(i-1)*skateLength/(numSegments-1), podWidth/2,-podHeight/2];
    end    
    skatePoints=[airSkateLeft airSkateRight];                
       
    % points for the rail wheels, while not extended
    wheelGap = .04; %m
    wheelVert = .05; %m   
    numWheels = 8;
    
    rightRailWheelPoints = zeros(3,numWheels);
    leftRailWheelPoints = zeros(3,numWheels);
    for i=1:numWheels
       rightRailWheelPoints(:,i) = [podLength/2-(i-.5)*podLength/numWheels,...
           -wheelGap/2, wheelVert-podHeight/2];
       leftRailWheelPoints(:, i) = [podLength/2-(i-.5)*podLength/numWheels,...
            wheelGap/2, wheelVert-podHeight/2];
    end
    
    %%%% for kalman filter %%%%
    
    covariance = zeros(10);
    
    % state is a 10x1 matrix of [position velocity quaternions]
    state = [transPos(:,1)' transVel(:,1)' q(:,1)']';
    
    kalmanHistory = zeros(10,numSteps/10);
    
    
    disp('Simulation Initialized')
    %%%BEGIN LOOPING THROUGH TIMESTEPS%%%
    for n = 2:numSteps
        if mod(n,1000) == 0
            disp('--------------------------')
            disp(n*timestep)
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
            if transPos(1,n-1) < PUSHER_DISTANCE
                localPusherForce = rotMatrix\[PUSHER_FORCE; 0; 0];
                localPusherPoint = [-podLength/2;0;0];
                
                localForces=[localForces localPusherForce];
                localPoints=[localPoints localPusherPoint];
            end
            
            
            %%%%% DRAG FORCE %%%%%
            drag = DRAG_COEFFICIENT* AIR_DENSITY*podWidth*podHeight*(transVel(1,n-1))^2 / 2;
            localDragForce = rotMatrix\[-drag;0;0];
            localDragPoint= [podLength/2; 0; 0];
            
            localForces=[localForces localDragForce];
            localPoints=[localPoints localDragPoint];
            
            %%%GRAVITY FORCE%%
            gravityForce=[0 0 -9.8]* mass;
            localGravityForce=rotMatrix\gravityForce';
            localGravityPoint=[0;0;0];

            localForces=[localForces localGravityForce];
            localPoints=[localPoints localGravityPoint];
           
        
        
        if randomNoise
            
            forceSize=size(localForces);
            noise=zeros(3,forceSize(2));
            for i=1:forceSize(2)
               magForce=norm(localForces(:,i));
               noise(:,i)=-1*noiseModifier+(2*noiseModifier*rand())*magForce;
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
        rotAcc(:,n) = I \ transpose(netTorque);
        
        %update omega, calculate omega matrix, update quaternions
        rotVel(:,n) = rotVel(:,n-1) + timestep * rotAcc(:,n);

        OmegaMatrix=[0 rotVel(3,n) -rotVel(2,n) rotVel(1,n);...
                    -rotVel(3,n) 0 rotVel(1,n) rotVel(2,n);...
                     rotVel(2,n) -rotVel(1,n) 0 rotVel(3,n);...
                    -rotVel(1,n) -rotVel(2,n) -rotVel(3,n) 0;];

        qdot = 0.5*OmegaMatrix*q(:,n-1);
                
        qn=q(:,n-1)+timestep*qdot;
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
        transAcc(:,n)=netForce/mass;
        transVel(:,n) = transVel(:,n-1) + transAcc(:,n)*timestep;
        transPos(:,n) = transPos(:,n-1) + transVel(:,n)*timestep;
        
        
        %%%%%%%%%%% KALMAN FILTER STEP %%%%%%%%%%%%%%%%%%%%
        
        % for bare bones implementation testing, we'll have no corrective
        % anything and perfect IMU data
        
        if mod(n,10) == 0
            
            %%%%% GET SENSOR DATA %%%%%
            
            % laser scanners
            scanner1Pos = [0; 0; 0];
            [~, scanner1dist] = DistanceFinder(rotMatrix*scanner1Pos); 
            
            scanner2Pos = [0; 0; 0];
            [~, scanner2dist] = DistanceFinder(rotMatrix*scanner2Pos); 
            
            scanner3Pos = [0; 0; 0];
            [~, scanner3dist] = InsideDistance(rotMatrix*scanner3Pos); 
            
            
            
            sensorData = [0 0 0 0 0 0 0];
            execution =  [0 0 0 0 0 0 0];

            IMUData = [transAcc(:,n)' rotVel(:,n)']';
            IMUData = IMUData + [0 0 9.81 0 0 0]';

            % add random noise
            IMUData = (.1*(rand(1)-.5)).*[1 1 1 1 1 1]' + IMUData;

            [state, covariance] = KalmanFilterHyperloop(state, covariance, IMUData, sensorData, execution);

            kalmanHistory(:,n/10) = state;
        end
        
        
        %check collisions
        endSimul=false;
        for point = collisionPoints';
            point = rotMatrix*point + transPos(:,n);
            [distance, ~]=DistanceFinder(point);
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