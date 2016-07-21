function [underSideDistance,beamDistance] = InsideDistance(point)
    %Using dimensions from subtrack 1

    % constants of the beam

    topHeight=2*.412; %where'd this come from?
    middleWidth=0.0079502;
    distanceToCenter=23.4; %where'd this come from?
       
    %Distances to beam
    underSideDistance=(-1)*(distanceToCenter+topHeight)-point(3);
    beamDistance=point(2)-(middleWidth/2);

          