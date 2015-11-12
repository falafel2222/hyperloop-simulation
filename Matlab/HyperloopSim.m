%Created by Alex Goldstein and Adam Shaw

function pd = HyperloopSim()
    disp('Simulation Started')
        
    %set initial variables
    % Tube conditions
    DISTANCETOFLAT=0.72136;
    DRAG_COEFFICIENT = 0.2;
    AIR_PRESSURE = 1.45;
    AIR_DENSITY = 6900*AIR_PRESSURE/(287*298);
    PUSHER_FORCE = 17640; % newtons
    PUSHER_DISTANCE = 243; % meters

    
    timestep = .001; %sec
    maxTime = 10; %sec
    numSteps = maxTime / timestep;
    
    
    %%%%%%%%%% CREATE THE POD AND SET ITS INITIAL CONDITIONS %%%%%%%%%%
   
    % Constructor: podData(mass,length, width, height, numSteps, timestep)
    pd = podData(1500,4,1,1,numSteps,timestep);
    
    %pod dimensions
    skateLength = pd.length; %m

    % start the pod with its initial height at idealStartHeight above the ground
    idealStartHeight=0.001;
    pd.transPos(3,1) = -1*(DISTANCETOFLAT-(pd.height/2)-idealStartHeight);
    
    %%% SET AIR THRUSTERS %%%
    numSegments = 2;
    pd.rightSkate=zeros(3,numSegments);
    pd.leftSkate=zeros(3,numSegments);
    for i=1:numSegments
       pd.rightSkate(:,i) = [pd.length/2-(i-.5)*skateLength/(numSegments),-pd.width/2,-pd.height/2];
       pd.leftSkate(:, i) = [pd.length/2-(i-.5)*skateLength/(numSegments), pd.width/2,-pd.height/2];
    end
    
    %%% SET RAIL WHEELS %%%
    wheelGap = .04; %m
    wheelVert = .05; %m   
    numWheels = 8;
    
    pd.rightRailWheels = zeros(3,numWheels);
    pd.leftRailWheels = zeros(3,numWheels);
    for i=1:numWheels
       pd.rightRailWheels(:,i) = [pd.length/2-(i-.5)*pd.length/numWheels,...
           -wheelGap/2, wheelVert-pd.height/2];
       pd.leftRailWheels(:, i) = [pd.length/2-(i-.5)*pd.length/numWheels,...
            wheelGap/2, wheelVert-pd.height/2];
    end
    
    %%%%% SET SENSORS %%%%%%
    
    
    % Laser (refreshRate (timesteps),error (%), location, direction)
    bottomFrontLaser = Laser(ceil(1e3*timestep),1e-3,...
                            [pd.length/2,0,-pd.height/2],[0,0,-1]);
    bottomBackLaser = Laser(ceil(1e3*timestep),1e-3,...
                            [-pd.length/2,0,-pd.height/2],[0,0,-1]);
    frontRightTrackLaser = Laser(ceil(1e3*timestep),1e-3,...
                            [pd.length/2,.1,wheelVert-pd.height/2],[0,-1,0]);
    frontLeftTrackLaser  = Laser(ceil(1e3*timestep),1e-3,...
                            [pd.length/2,.1,wheelVert-pd.height/2],[0,-1,0]);
    backRightTrackLaser  = Laser(ceil(1e3*timestep),1e-3,...
                            [pd.length/2,-.1,wheelVert-pd.height/2],[0,1,0]);
    backLeftTrackLaser   = Laser(ceil(1e3*timestep),1e-3,...
                            [pd.length/2,-.1,wheelVert-pd.height/2],[0,1,0]);
    
    lasers = [bottomFrontLaser bottomBackLaser frontRightTrackLaser
        frontLeftTrackLaser backRightTrackLaser backLeftTrackLaser];
                        
    photoElectricSensor = Sensor(1e2,0);
    pitotTube = Sensor(1e2,1e-3);
    
    
    
    
    
    
    
    %%%%% OTHER SENSORS %%%%%
    
    disp('Simulation Initialized')
    %%%BEGIN LOOPING THROUGH TIMESTEPS%%%
    for n = 2:numSteps
        if mod(n,1000) == 0
            disp('--------------------------')
            disp(n*timestep)
            disp(pd.transPos(:,n-1)')
%             disp(pd.transAcc(:,n-1)')
%             disp(pd.q(:,n-1))
        end

           
           %%%%% AIR SKATES %%%%%

           for i=1:length(pd.rightSkate(1,:))
               % calculate vertical distance and find the force accordingly 
               [~, vertDist]=DistanceFinder(pd.toGlobal(pd.rightSkate(:,i)));
               f = Force(true,pd.rightSkate(:,i),[0;0;SkateForce(vertDist,11e3,skateLength)],0);
               pd = pd.applyForce(f);

               % same for the left skate
               [~, vertDist]=DistanceFinder(pd.toGlobal(pd.leftSkate(:,i)));
               f = Force(true,pd.leftSkate(:,i),[0;0;SkateForce(vertDist,11e3,skateLength)],0);
               pd = pd.applyForce(f);
           end
            
            %%%%% RAIL WHEELS %%%%%
            % will do later when we get better estimates
             
             
            %%%%% SPACEX PUSHER %%%%%
            if pd.transPos(1,n-1) < PUSHER_DISTANCE
                globalPusherForce = [PUSHER_FORCE; 0; 0];
                localPusherPoint = [-pd.length/2;0;0];
                pusherForce = Force(false,localPusherPoint,globalPusherForce,.01);
                pd = pd.applyForce(pusherForce);
            end
            
            
            %%%%% DRAG FORCE %%%%%
            drag = DRAG_COEFFICIENT* AIR_DENSITY*pd.width*pd.height*(pd.transVel(1,n-1))^2 / 2;

            globalDragForce = [-drag;0;0];
            localDragPoint= [pd.length/2; 0; 0];            
            pd = pd.applyForce(Force(false,localDragPoint,globalDragForce,0));
            
            %%%%% GRAVITY FORCE %%%%%
            globalGravityForce=[0; 0; -9.8]* pd.mass;
            localGravityPoint=[0;0;0];
            pd = pd.applyForce(Force(false,localGravityPoint,globalGravityForce,0));
        
        
        %%%%% GET NEW SENSOR DATA %%%%%
        
%             for laser = lasers
%                 if mod(n,laser.refreshRate) == 0
%                     % calculate laser distance and get ready to pass to
%                     % states
%                 end                
%             end
            
            % get other sensors from sensors team and add them

        pd = pd.update();
        
        %calculate roll, pitch, and yaw
%         rotPos(:,n-1) = [atan2(2*(q0*q1+q2*q3),1-2*(q1^2 + q2^2)); ...
%                          asin(2*(q0*q2 - q3*q1));...
%                          atan2(2*(q0*q3+q1*q2),1-2*(q2^2 + q3^2))];

        
        %check collisions
        endSimul=false;
        for point = pd.collisionPoints;
            point = pd.rotMatrix*point + pd.transPos(:,n);
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
end