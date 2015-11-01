function [radialDistance, vertDist] = DistanceFinder(point)
    % constants of the tube
    alphaAngle=3*pi/2-acos(23.0/35);
    betaAngle=3*pi/2+acos(23.0/35);
    stdRadius=0.889;
    flatHeight=-0.5842;
    
    % finding radius of the tube at any given theta
    zVal=point(3);
    yVal=point(2);
    theta=atan2(zVal,yVal);
    if theta>alphaAngle && theta<betaAngle
       gammaAngle=3*pi/2-theta;
    else
       gammaAngle=0;
    end
    radiusAtTheta=stdRadius/cos(gammaAngle);
    radiusOfPoint=sqrt(zVal^2+yVal^2);

    radialDistance=radiusAtTheta-radiusOfPoint;
    vertDist=min([radialDistance*cos(theta-3*pi/2), zVal-flatHeight]);
    