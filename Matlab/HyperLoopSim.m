% Authors: Adam Shaw, Alex Goldstein, Patrick McKeen
% Simulation of the hyperloop pod

%DEPRECATED, REPLACED BY HYPERLOOPSIMV2

function [] = HyperLoopSim()

% global variables
timestep = .01;
maxTime = 50;
numSteps = maxTime / timestep;

% pod state variables
pos = zeros(3, numSteps);
vel = zeros(3, numSteps);
acc = zeros(3, numSteps);

rotPos = zeros(3, numSteps);
rotVel = zeros(3, numSteps);
rotAcc = zeros(3, numSteps);

collisionPointsTrackFrame = zeros(numSteps, 8,3);

q=zeros(4, numSteps);
q(:,1)=[0.00199999; 0.00199999; 0.00199999; 0.999994];

% pod constants
podHeight = 1.0;
podWidth = 1.0;
podLength = 4.0;

mass = 3000;
CoM = [0,0,0];

% air thrusters are on the bottom of the pod and fire downwards
airThrusters =  [podLength/2   podWidth/2  -podHeight/2-CoM(3); ...
                 podLength/2  -podWidth/2  -podHeight/2-CoM(3); ...
                -podLength/2   podWidth/2  -podHeight/2-CoM(3); ...
                -podLength/2  -podWidth/2  -podHeight/2-CoM(3)];

% collision points: where the pod might intersect the track
collisionPoints = [podLength/2   podWidth/2  -podHeight/2-CoM(3); ...
                   podLength/2  -podWidth/2  -podHeight/2-CoM(3); ...
                  -podLength/2   podWidth/2  -podHeight/2-CoM(3); ...
                  -podLength/2  -podWidth/2  -podHeight/2-CoM(3); ...
                   podLength/2   podWidth/2   podHeight/2-CoM(3); ...
                   podLength/2  -podWidth/2   podHeight/2-CoM(3); ...
                  -podLength/2   podWidth/2   podHeight/2-CoM(3); ...
                  -podLength/2  -podWidth/2   podHeight/2-CoM(3)];

% Code to find tensor of inertia
% will be updated once the CAD is actually ready, right now it's a cuboid
I = [1.0/12*mass*(podHeight^2 + podWidth^2) 0 0; ...
     0 1.0/12*mass*(podHeight^2 + podLength^2) 0; ...
     0 0 1.0/12*mass*(podLength^2 + podWidth^2)];


for n = 2:numSteps    
    
    % we need the rotational matrix to convert between local and global
    q0=q(4,n-1);
    q1=q(1,n-1);
    q2=q(2,n-1);
    q3=q(3,n-1);
    rotMatrix=[1-2*q2^2-2*q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
               2*(q1*q2+q0*q3) 1-2*q1^2-2*q3^2 2*(q2*q3-q0*q1);...
               2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*q1^2-2*q2^2];    
    
           
    % control algorithm - decide how hard to fire thrusters
    % for now let's be dumb.
    thrusterForces = transpose([0 0 mass*2.45 + rand(1);...
                                0 0 mass*2.45 + rand(1);...
                                0 0 mass*2.45 + rand(1);...
                                0 0 mass*2.45 + rand(1)]);
    
    % calculate translational forces
    
    acc(:,n) = [0 0 0];
    
    % gravity
    acc(:,n) = acc(:,n) + [0; 0; -9.8];
    
    for i = 1:size(airThrusters,1)
        % these are currently in the pod frame - change to global
        thrusterForce = rotMatrix * (thrusterForces(:,i));
        acc(:,n) = acc(:,n) + thrusterForce/mass;
    end
        
    % update pos and vel based on acc
    vel(:,n) = vel(:,n-1) + acc(:,n)*timestep;
    pos(:,n) = pos(:,n-1) + vel(:,n)*timestep;
    
    % calculate rotational forces
    
    % convert all forces to local and calculate angular acceleration
    netTorque = [0 0 0];
    for i = 1:size(airThrusters,1)
        % air thrusters are already in local
        thrusterTorque = cross(airThrusters(i,1:3),transpose(thrusterForces(:,i)));
        netTorque = netTorque + thrusterTorque;
    end
    
    rotAcc(:,n) = I \ transpose(netTorque);

    rotVel(:,n) = rotVel(:,n-1) + timestep * rotAcc(:,n);
    
    % update quaternions - math I don't really understand too well
    OmegaMatrix=[0 rotVel(3,n) -rotVel(2,n) rotVel(1,n);...
                 -rotVel(3,n) 0 rotVel(1,n) rotVel(2,n);...
                 rotVel(2,n) -rotVel(1,n) 0 rotVel(3,n);...
                 -rotVel(1,n) -rotVel(2,n) -rotVel(3,n) 0;];
                 
    qn=q(:,n-1)+0.5*OmegaMatrix*q(:,n-1);
    normQuat=sqrt(sum(qn.^2));
    qn=qn./normQuat;
    normQuat=sqrt(sum(qn.^2));
       
    % vestigial from Patrick's debugging, I think
    if ((normQuat>1.00000001) || (normQuat<0.99999999))
        fprintf('WARNING, quaternions norm doesnt equal 1!!' + qn);
        break
    end
    q(:,n)=qn;
    
    
    for i = 1:size(collisionPoints, 1)
        rotatedPoint = rotMatrix*transpose(collisionPoints(i,:));
        collisionPointsTrackFrame(n,i,:) = rotatedPoint;
    end
    
    %collisionOccurred=CollisionChecking(squeeze(collisionPointsTrackFrame(n,:,:)));
    %if collisionOccurred==1
    %    disp('Collision Occurred!')
    %    break
    %end
end

% done with the simulation - time to graph things!

timeArray = 1:numSteps;
timeArray = timeArray * timestep;

%plot3(collisionPointsTrackFrame(2:n,:,1),collisionPointsTrackFrame(2:n,:,2),collisionPointsTrackFrame(2:n,:,3));

 
 for i = 2:10:n
     
     % needs to be revised to allow for proper bounds checking
     % when combining pos and rotation.
%    A = transpose(squeeze(collisionPointsTrackFrame(i,5,:)) + pos(:,i));
%    B = transpose(squeeze(collisionPointsTrackFrame(i,1,:)) + pos(:,i));
%    C = transpose(squeeze(collisionPointsTrackFrame(i,6,:)) + pos(:,i));
%    D = transpose(squeeze(collisionPointsTrackFrame(i,7,:)) + pos(:,i));
%    E = transpose(squeeze(collisionPointsTrackFrame(i,8,:)) + pos(:,i));
%    F = transpose(squeeze(collisionPointsTrackFrame(i,3,:)) + pos(:,i));
%    G = transpose(squeeze(collisionPointsTrackFrame(i,2,:)) + pos(:,i));
%    H = transpose(squeeze(collisionPointsTrackFrame(i,4,:)) + pos(:,i));

%    P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B]; 
    
%    plot3(P(:,1),P(:,2),P(:,3));
    
%    zlim([-200 100]);
%    ylim([-200 200]);
%    xlim([-200 200]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % plots position only based on rotations
    A = collisionPointsTrackFrame(i,5,:);
    B = collisionPointsTrackFrame(i,1,:);
    C = collisionPointsTrackFrame(i,6,:);
    D = collisionPointsTrackFrame(i,7,:);
    E = collisionPointsTrackFrame(i,8,:);
    F = collisionPointsTrackFrame(i,3,:);
    G = collisionPointsTrackFrame(i,2,:);
    H = collisionPointsTrackFrame(i,4,:);

    P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B]; 
    
    plot3(P(:,1),P(:,2),P(:,3));
    
    zlim([-2 2]);
    ylim([-2 2]);
    xlim([-2 2]);


    pause(.01);
 end     

% plot(pos(3,:));