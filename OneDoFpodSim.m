function [] = OneDoFpodSim(runtime, Apusher, Aresist, realFpb, realFeb, Fpb, Mpod  )


dt = .001; %1kHz
numdatapoints = runtime/dt;
t = 0.001*1:numdatapoints; %1kHz


%construct empty arrays for plotting
XP = zeros(1,numdatapoints); %postion
XV = zeros(1,numdatapoints); %velocity
XA = zeros(1,numdatapoints); %acceleration

xpos = 0;
xvel = 0;
xaccel = Apusher + Aresist;

%Standby
PodPhase = 0;
BrakingModeStatus = 0;
disp 'Standby Mode Initialized'
%make more rigorous (update variables and run a few times...
PhaseAndBraking(PodPhase,BrakingModeStatus,[0 ,1],[0 0 xaccel],[xpos 0 0 xvel 0 0 0 0 0],0,100,0,20000,Fpb,realFeb,1600,[0, 0, 0, 1, 0],300,2000,750,Aresist,19.62,5,5,.05,5,5)
disp 'Standby Complete'

%Preflight
%Switch values to preflight
PodPhase = 1;
BrakingModeStatus = 0;
disp 'Exiting Standby Mode... Entering PreFlight checks'
%run PhaseAndBraking a few times, print statements each time...
out = PhaseAndBraking(PodPhase,BrakingModeStatus,[0 ,1],[0 0 xaccel],[xpos 0 0 xvel 0 0 0 0 0],0,100,0,20000,Fpb,realFeb,1600,[0, 0, 0, 1, 0],300,2000,750,Aresist,19.62,5,5,.05,5,5); 
BrakingModeStatus = out(2)
disp 'Cycle 1'
out = PhaseAndBraking(PodPhase,BrakingModeStatus,[0 ,1],[0 0 xaccel],[xpos 0 0 xvel 0 0 0 0 0],0,100,0,20000,Fpb,realFeb,1600,[0, 0, 0, 1, 0],300,2000,750,Aresist,19.62,5,5,.05,5,5);
BrakingModeStatus = out(2)
disp 'Cycle 2'
out = PhaseAndBraking(PodPhase,BrakingModeStatus,[0 ,1],[0 0 xaccel],[xpos 0 0 xvel 0 0 0 0 0],0,100,0,20000,Fpb,realFeb,1600,[0, 0, 0, 1, 0],300,2000,750,Aresist,19.62,5,5,.05,5,5);
BrakingModeStatus = out(2)
disp 'Cycle 3'
out = PhaseAndBraking(PodPhase,BrakingModeStatus,[0 ,1],[0 0 xaccel],[xpos 0 0 xvel 0 0 0 0 0],0,100,0,20000,Fpb,realFeb,1600,[0, 0, 0, 1, 0],300,2000,750,Aresist,19.62,5,5,.05,5,5);
BrakingModeStatus = out(2)
disp 'Cycle 4'
out = PhaseAndBraking(PodPhase,BrakingModeStatus,[0 ,1],[0 0 xaccel],[xpos 0 0 xvel 0 0 0 0 0],0,100,0,20000,Fpb,realFeb,1600,[0, 0, 0, 1, 0],300,2000,750,Aresist,19.62,5,5,.05,5,5);
BrakingModeStatus = out(2)
disp 'Cycle 5'
disp 'PreFlight Checks Complete'

%PusherPhase
%Switch Values to PusherPhase
PodPhase = 2;
BrakingModeStatus = 9;
disp 'POD LAUNCHING'
PlantStatus = [0, 0, 0, 0, 1];
%Pod Run (After PusherPhase initiated)
for i = 2:numdatapoints
    co = i; %iteration counter
    Mpod = Mpod - 0.2/1000;


    %Left Side Reimmans (uses last step acceleration)
    %x-updates:
    xvel = xvel + xaccel*dt; %update x-velocity (m/s)
    xpos = xpos + xvel*dt; %update x-position (m)
    
    if xvel < 0 %end of run
        disp 'POD STOPPED'
        i = numdatapoints;
        break
    end

    PodPhases = {'Preflight','Pusher Phase','Coasting Phase','Braking Phase'};
    BrakingModeStatuses = {'1','2','3','4','Full Stop','Full Stop Less','Primary Stop','Braking Ready','Off'};
    
    disp 'PodPhase is'
    if PodPhase == 1
        disp 'ERROR: Preflight'
        i = numdatapoints;
        break
    elseif PodPhase == 2
        disp 'Pusher Phase'
    elseif PodPhase == 3
        disp 'Coasting Phase'
        Apusher = 0;
    elseif PodPhase == 4
        disp 'Braking Phase'
        Apusher = 0;
    else
        disp 'ERROR: no PodPhase (or is Standby)...'
        i = numdatapoints;
        break
    end
    
    disp 'BreakingModeStatus is'
    if BrakingModeStatus == 5
        disp 'Full Stop'
        currentrealFpb = realFpb;
        currentrealFeb = realFeb;
    elseif BrakingModeStatus == 6
        disp 'Full Stop Less'
        currentrealFpb = 0;
        currentrealFeb = realFeb;
    elseif BrakingModeStatus == 7
        disp 'Primary Stop'
        currentrealFpb = realFpb;
        currentrealFeb = 0;
    elseif BrakingModeStatus == 8
        disp 'Braking Ready'
        currentrealFpb = 0;
        currentrealFeb = 0;
    elseif BrakingModeStatus == 9
        disp 'Off'
        currentrealFpb = 0;
        currentrealFeb = 0;
    else
        disp 'ERROR: no BrakingModeStatus (or is Standby)...'
        i = numdatapoints;
        break
    end
    
    %update xaccel
    xaccel = Apusher + Aresist + currentrealFpb/Mpod + currentrealFeb/Mpod;

    %build x-motion arrays;
    XP(1,i) = xpos;
    XV(1,i) = xvel;
    XA(1,i) = xaccel;

    %Calculate inputs:
    %PodPhase
    %BrakingModeStatus
    CommandPointCommands = [0 ,1];
    IMUglobaldata = [0 0 xaccel];
    KalmanStates = [xpos 0 0 xvel 0 0 0 0 0];
    StripCount = 0; %idk
    BrakingStripThreshold = 100; %arb
    FlightTime = co; %ms
    BrakingTimeThreshold = 20*1000; %ms
    %Fpb
    Feb = realFeb; %accurate estimation
    TrackLength = 1600; %m
    %PlantStatus
    PusherReleaseDist = 300; %m arb
    PusherReleaseTime = 2*1000; %ms arb
    %Mpod
    %Aresist
    Amax = 9.81*2; %m/s^2
    ReleaseDistMargin = 5; %m
    PrimaryBrakeStoppingDistanceMargin = 5; %m
    PBerrorMargin = .1; %5percent 
    CurrentStoppingDistanceMargin = 5; %m
    FullBrakeSoppingDistanceMargin = 5; %m
    
    %Run PhaseAndBraking and save results
    result = PhaseAndBraking(PodPhase,BrakingModeStatus,CommandPointCommands,IMUglobaldata,KalmanStates,StripCount,BrakingStripThreshold,FlightTime,BrakingTimeThreshold,Fpb,Feb,TrackLength,PlantStatus,PusherReleaseDist,PusherReleaseTime,Mpod,Aresist,Amax,ReleaseDistMargin,PrimaryBrakeStoppingDistanceMargin,PBerrorMargin,CurrentStoppingDistanceMargin,FullBrakeSoppingDistanceMargin);
    
    %Assign new values from saved output
    PodPhase = result{1};
    BrakingModeStatus = result{2};
%     PrimaryBrakeCommand = result{3};
%     FrontEBrakeCommand = result{4};
%     BackEBrakeCommand = result{5};
%     WheelsCommand = result{6};
%     SkatesCommand = result{7};
    Fpb = result{8};
    disp '...'
end

trim = 1;
co = co-trim;
 
close all
subplot(2,1,1);
plot(XP(1:co),XA(1:co),'r');
title('x-acceleration vs. x-position');
xlabel('m');
ylabel('m/s^2');

subplot(2,1,2);
plot(XP(1:co),XV(1:co),'b');
title('x-velocity vs. x-position');
xlabel('m');
ylabel('m/s');

end
