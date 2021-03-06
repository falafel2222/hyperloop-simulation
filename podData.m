classdef podData
    properties
        mass
        COM
        tensor
        length
        width
        height
        collisionPoints
        leftRailWheels
        rightRailWheels
        airskate
        skateSegmentLength
        eBrakeForce
        dragCoef
        skateHeight
        sideSensorDistance
        downSensorOffset
        bottomDistancePositions
        downRailDistancePositions
        sideDistancePositions
        pitotPosition
        topPhotoElectricPositions
        leftPhotoElectricPositions
        rightPhotoElectricPositions
        bottomDistanceDirections
        downRailDistanceDirections
        sideDistanceDirections
        pitotDirection
        topPhotoElectricDirections
        leftPhotoElectricDirections
        rightPhotoElectricDirections
        photoElectricTilt
        angleOfPESensitivity
        peMax
        peToWall
        peFloor
        
    end
    methods
        function pod = podData()
            pod.mass = 750;
            pod.COM = [-1.5904;0;0]; %%%%%% THIS IS WRONG, BUT WE DON'T HAVE A CONTROL ALGORITHM YET
            pod.length = 6;
            pod.width = 1;
            pod.height = 1;
            pod.photoElectricTilt=13*pi/180; %rad
            pod.angleOfPESensitivity = (40/2)*pi/180; %rad
            pod.peMax=1;
            pod.peFloor=0.1;
            pod.skateHeight=0.113919;
            pod.downSensorOffset=0.0005;
            pod.sideSensorDistance=6*2.54/100;
            pod.tensor=[   1.0/12*pod.mass*(pod.height^2 + pod.width^2) 0 0; ...
                            0 1.0/12*pod.mass*(pod.height^2 + pod.length^2) 0; ...
                            0 0 1.0/12*pod.mass*(pod.length^2 + pod.width^2)];
                   
              pod.collisionPoints =[[pod.length/2 pod.width/2 -pod.skateHeight]'...     
                                    [pod.length/2 -pod.width/2 -pod.skateHeight]'...
                                    [-pod.length/2 pod.width/2 -pod.skateHeight]'...
                                    [-pod.length/2 -pod.width/2 -pod.skateHeight]'...
                                    [pod.length/3 pod.width/2 (pod.height*0.5)-pod.skateHeight]'...
                                    [pod.length/3 -pod.width/2 (pod.height*0.5)-pod.skateHeight]'...
                                    [-pod.length/3 -pod.width/2 (pod.height*0.5)-pod.skateHeight]'...
                                    [-pod.length/3 -pod.width/2 (pod.height*0.5)-pod.skateHeight]'...
                                    [pod.length/3 0 (pod.height)-pod.skateHeight]'...
                                    [-pod.length/3 0 (pod.height)-pod.skateHeight]'];
%             pod.collisionPoints =[[1.04305	-0.08234	-0.06705]'...
%                                   [1.04305	 0.08276	-0.06705]'...
%                                   [1.00203	-0.09498	-0.09674]'...
%                                   [1.00203	 0.09498	-0.09674]'...
%                                   [-4.31659	-0.0951     -0.09669]'...
%                                   [-4.31659	 0.0951     -0.09669]'...
%                                   [0.25835	-0.4064 	-0.11263]'...
%                                   [0.25835	 0.4064 	-0.11263]'...
%                                   [-3.45005	-0.4064 	-0.11263]'...
%                                   [-3.45005	 0.4064 	-0.11263]'...
%                                   [0.25835	 0.0762 	-0.11263]'...
%                                   [0.25835	-0.0762 	-0.11263]'...
%                                   [-3.45005	 0.0762 	-0.11263]'...
%                                   [-3.45005 -0.0762 	-0.11263]'];

            pod.skateSegmentLength = 1.14;
            pod.dragCoef = 2.7;
                    
            % GENERATE SKATE POINTS

            
            
            skateCoordinates = [
                [-3.3917382	-0.3556	-0.113919]'...
                [-2.16408	-0.3556	-0.113919]'...
                [-0.9364218	-0.3556	-0.113919]'...
                [-3.3917382	 0.127	-0.113919]'...
                [-2.16408	 0.127	-0.113919]'...
                [-0.9364218	 0.127	-0.113919]'];

            pod.airskate = zeros(6,4,3); 
    
            for i = 1:6
               % each segment has 10x46 holes 1" apart - simulate as fewer
               % holes for increases speed
               segment = zeros(4,3);
               for x = 0:45:45
                   for y = 0:9:9
                       segment(x/45*2 + y/9+1,:) = skateCoordinates(:,i) + .0254*[x;y;0];
%                        display(segment(x*10 + y+1,:))
                   end
                   
               end
               pod.airskate(i,:,:) = segment;
            end

            pod.eBrakeForce = 0.8*9.8*pod.mass;
            
            %%sensor locations
            pod.bottomDistancePositions=[0 0 pod.length/2 pod.length/2 -pod.length/2 -pod.length/2;pod.width/2 -pod.width/2 pod.width/2 -pod.width/2 pod.width/2 -pod.width/2;pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight];
            pod.downRailDistancePositions=[pod.length/2 pod.length/3 0 -pod.length/3 -pod.length/2;0 0 0 0 0;pod.downSensorOffset  pod.downSensorOffset pod.downSensorOffset pod.downSensorOffset pod.downSensorOffset];
            pod.sideDistancePositions=[pod.length/2 pod.length/3 0 -pod.length/3 -pod.length/2;pod.sideSensorDistance pod.sideSensorDistance pod.sideSensorDistance pod.sideSensorDistance pod.sideSensorDistance;-pod.skateHeight/2 -pod.skateHeight/2 -pod.skateHeight/2 -pod.skateHeight/2 -pod.skateHeight/2];
            pod.pitotPosition=[pod.length/2;0;pod.height*2/3];
            pod.topPhotoElectricPositions=[pod.length*1/3 0 -pod.length/3; 0 0 0; pod.height-pod.skateHeight pod.height-pod.skateHeight pod.height-pod.skateHeight];
            pod.leftPhotoElectricPositions=[pod.length*1/3 0 -pod.length/3; pod.width/(2*sqrt(2)) pod.width/(2*sqrt(2)) pod.width/(2*sqrt(2)); (pod.height*(0.5+1/(2*sqrt(2)))-pod.skateHeight) (pod.height*(0.5+1/(2*sqrt(2)))-pod.skateHeight) (pod.height*(0.5+1/(2*sqrt(2)))-pod.skateHeight)];
            pod.rightPhotoElectricPositions=[pod.length*1/3 0 -pod.length/3; -pod.width/(2*sqrt(2)) -pod.width/(2*sqrt(2)) -pod.width/(2*sqrt(2)); (pod.height*(0.5+1/(2*sqrt(2)))-pod.skateHeight) (pod.height*(0.5+1/(2*sqrt(2)))-pod.skateHeight) (pod.height*(0.5+1/(2*sqrt(2)))-pod.skateHeight)];
            
            %%Sensor directions
            pod.bottomDistanceDirections=[0 0 0 0 0 0; 0 0 0 0 0 0; -1 -1 -1 -1 -1 -1;];
            pod.downRailDistanceDirections=[0 0 0 0 0 0; 0 0 0 0 0 0; -1 -1 -1 -1 -1 -1;];
            pod.sideDistanceDirections=[0 0 0 0 0 0;  -1 -1 -1 -1 -1 -1; 0 0 0 0 0 0;];
            pod.pitotDirection=[1;0;0];
            pod.topPhotoElectricDirections=[0 0 0; 0 0 0; 1 1 1;];
            pod.leftPhotoElectricDirections=[0 0 0; 1 1 1; 1 1 1;]/sqrt(2);
            pod.rightPhotoElectricDirections=[0 0 0; -1 -1 -1; 1 1 1;]/sqrt(2);
            
            pod.peToWall=[.5 .5 .5 .6 .6 .6 .6 .6 .6]'; %m THIS NEEDS TO BE CORRECTED
            
            
            
            
        end
        
    end
end

    