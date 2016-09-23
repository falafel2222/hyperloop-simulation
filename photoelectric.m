function [photoElectricCount, photoElectricUse,lastPE,prevPE,crossIn]=photoelectric(peCount,peData,lastPE,prevPE,crossIn,time,pod)
    photoElectricUse=[0 0 0]; %[max(peData(:,1)>pod.peFloor) max(peData(:,2)>pod.peFloor) max(peData(:,3)>pod.peFloor)]';
    
    data=[peData(1:3,1); peData(1:3,2); peData(1:3,3);];
    detectionMatrix=([data prevPE]>pod.peFloor);
%     (detectionMatrix*[1;2;4;8])==8
    peCount=peCount+((detectionMatrix*[1;2;4;8])==8);
    
    crossIn((detectionMatrix*[1;2;4;8])==3)=time;
     
    A=[peCount...
        [peCount(2) peCount(1)-1 peCount(1)-1 peCount(5) peCount(4)-1 peCount(4)-1 peCount(8) peCount(7)-1 peCount(7)-1]' ...
        [peCount(3) peCount(3) peCount(2)-1 peCount(6) peCount(6) peCount(5)-1 peCount(9) peCount(9) peCount(8)-1]'];
    
    photoElectricCount=max(A,[],2);
    
    ind=((detectionMatrix*[1;2;4;8])==12);
    lastPE(ind)=(crossIn(ind)+time)/2;
    %this should be updataed to include the "resets" that occur when there
    %are a lot of strips close to each other.
    prevPE(:,2:3)=prevPE(:,1:2);
    prevPE(:,1)=data;
end

