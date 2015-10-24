%Authors: Adam Shaw, Alex Goldstein
%Separate program to check if any points have collided with the walls of
%the tube

function [collisionOccurred] = CollisionChecking(points)

% constants of the tube
alphaAngle=3*pi/2-acos(23.0/35);
betaAngle=3*pi/2+acos(23.0/35);


% finding radius of the tube at any given theta
collisionOccurred=false;
for point = transpose(points)
    zVal=point(2);
    yVal=point(1);
    theta=atan2(zVal,yVal);
    if theta>alphaAngle && theta<betaAngle
       gammaAngle=3*pi/2-theta;
    else
       gammaAngle=0;
    end
    radiusAtTheta=0.889/cos(gammaAngle); %0.889 is 35 in. in m.
    radiusOfPoint=sqrt(zVal^2+yVal^2);
    if radiusOfPoint>=radiusAtTheta
       collisionOccurred=true;
       break
    end
end 
