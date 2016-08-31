function [photoElectricCount, photoElectricUse,lastPE]=photoelectric(peCount,peData,lastPE,prevPE,time,pod)
    photoElectricUse=[max(peData(:,1)>pod.peFloor) max(peData(:,2)>pod.peFloor) max(peData(:,3)>pod.peFloor)]';
     
    A=[peCount...
        [peCount(2) peCount(1)-1 peCount(1)-1 peCount(5) peCount(4)-1 peCount(4)-1 peCount(8) peCount(7)-1 peCount(7)-1]' ...
        [peCount(3) peCount(3) peCount(2)-1 peCount(6) peCount(6) peCount(5)-1 peCount(9) peCount(9) peCount(8)-1]'];
    
    photoElectricCount=max(A,[],2);
    
    ind=[peData(1:3,1); peData(1:3,2); peData(1:3,3);];
    lastPE(ind>pod.peFloor)=time;
    %this should be updataed to include the "resets" that occur when there
    %are a lot of strips close to each other.
end

