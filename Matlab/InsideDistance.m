function [underSideDistance,beamDistance] = InsideDistance(point)
    % constants of the beam

    topHeight=0.88;
    middleWidth=0.23;
    distanceToCenter=23;
       
    %Distances to beam
    underSideDistance=(-1)*(distanceToCenter+topHeight)-point(3);
    beamDistance=abs(point(2))-(middleWidth/2);

          