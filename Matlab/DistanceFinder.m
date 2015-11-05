function [radialDistance, vertDist] = DistanceFinder(point)
    
    %All values for subtrack option 1

    % constants of the tube
    tubeRadiusM=0.889;
    distToFlatM=-0.72136;
    alphaAngle=3*pi/2-acos(distToFlatM/tubeRadiusM);
    betaAngle=3*pi/2+acos(distToFlatM/tubeRadiusM);
    
    % finding radius of the tube at any given theta
    zVal=point(3);
    yVal=point(2);
    theta=atan2(zVal,yVal);
    if theta>alphaAngle && theta<betaAngle
       gammaAngle=3*pi/2-theta;
    else
       gammaAngle=0;
    end
    radiusAtTheta=tubeRadiusM/cos(gammaAngle);
    radiusOfPoint=sqrt(zVal^2+yVal^2);

    radialDistance=radiusAtTheta-radiusOfPoint;
    vertDist=min([radialDistance*cos(theta-3*pi/2), zVal-distToFlatM]);
    