%Created by Alex Goldstein and Adam Shaw

function [] = HyperloopSimV2()
    disp('Simulation Started')
    %set initial variables
        %pod dimensions
        podHeight = 1; %m
        podWidth = 1; %m
        podLength = 1.3; %m
        skateLength = podLength; %m
    
        mass = 3000; %kg
        I = [1.0/12*mass*(podHeight^2 + podWidth^2) 0 0; ...
             0 1.0/12*mass*(podHeight^2 + podLength^2) 0; ...
             0 0 1.0/12*mass*(podLength^2 + podWidth^2)];
        CoM = [0,0,0];
        
        timestep = .001; %sec
        maxTime = 10; %sec
        numSteps = maxTime / timestep;
        
        % pod state variables
        transPos = zeros(3, numSteps);
        transVel = zeros(3, numSteps);
        transAcc = zeros(3, numSteps);
        
        rotPos = zeros(3, numSteps);
        rotVel = zeros(3, numSteps);
        rotAcc = zeros(3, numSteps);
        
        q=zeros(4, numSteps);
        q(:,1)=[0.00199999; 0.00199999; 0.00199999; 0.999994];
        


        
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
    airSkateRight=zeros(3,10);
    airSkateLeft=zeros(3,10);
    for i=1:length(airSkateRight)
       airSkateRight(:,i+1) = [podLength/2-i*skateLength/10,-podWidth/2,-podHeight/2];
       airSkateLeft(:, i+1) = [podLength/2-i*skateLength/10, podWidth/2,-podHeight/2];
    end
    skatePoints=[airSkateLeft airSkateRight];                
    
    disp('Simulation Initialized')
    %%%BEGIN LOOPING THROUGH TIMESTEPS%%%
    for n = 2:numSteps
        if mod(n,100)==0
            disp(n/1000)
            disp(transPos(:,n))
        end
        
        
        %calculate center of mass, tensor of inertia
        
        %find quaternions based on previous quaternion values
         q0=q(4,n-1);
         q1=q(1,n-1);
         q2=q(2,n-1);
         q3=q(3,n-1);
         
        %calculate rotation matrix
        rotMatrix=[1-2*q2^2-2*q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
                   2*(q1*q2+q0*q3) 1-2*q1^2-2*q3^2 2*(q2*q3-q0*q1);...
                   2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*q1^2-2*q2^2];
                
        %get forces in local
        localForces=[];
        localPoints=[];
           %forces already in local
            skateForces=zeros(3,length(skatePoints));
          
            for i=1:length(skatePoints)
                point=skatePoints(:,i);
                [~, vertDist]=DistanceFinder(point);
                
                %%%%THIS LINE ISN'T RIGHT, STILL SAME MAGNITUDE%%%%
                vertDist=rotMatrix*vertDist;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                %pass 0 internal pressure for now
                pointForce=SkateForce(vertDist(3),0);
                skateForces(:,i)=pointForce/length(airSkateRight);
            end
            localForces=[localForces skateForces];
            localPoints=[localPoints skatePoints];
             
            %forces in global
            
            %%%GRAVITY IS EXAMPLE, ACTUALLY TRIVIAL%%%
            gravityForce=[0 0 -9.8];
            gravityPoint=[0 0 0];
            localGravityForce=rotMatrix\gravityForce';
            localGravityPoint=rotMatrix\gravityPoint';
           
            localForces=[localForces localGravityForce];
            localPoints=[localPoints localGravityPoint];
           
            
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
                    
            qn=q(:,n-1)+0.5*OmegaMatrix*q(:,n-1);
            normQuat=sqrt(sum(qn.^2));
            qn=qn./normQuat;
            normQuat=sqrt(sum(qn.^2));
            
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
        if mod(n,1000)==0
            globalForces
        end
        
        %get accel, velocity, position
        transAcc(:,n)=netForce/mass;
        transVel(:,n) = transVel(:,n-1) + transAcc(:,n)*timestep;
        transPos(:,n) = transPos(:,n-1) + transVel(:,n)*timestep;
        
        %check collisions
        endSimul=false;
        for point=collisionPoints';
            [distance, ~]=DistanceFinder(point);

            if distance<=0
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
    
    
       