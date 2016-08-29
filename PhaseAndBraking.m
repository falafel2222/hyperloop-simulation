function [ output ] = PhaseAndBraking( PodPhase,BrakingModeStatus,CommandPointCommands,IMUglobaldata,KalmanStates,StripCount,BrakingStripThreshold,FlightTime,BrakingTimeThreshold,Fpb,Feb,TrackLength,PlantStatus,PusherReleaseDist,PusherReleaseTime,Mpod,Aresist,Amax )
%OpenLoop
%SpaceX Hyperloop Pod Competition 2015-2016
%Control - Braking Algorithm
%Version 0.9
%Updated: 08/28/2016
%Chris Kotcherha - Harvey Mudd College - Class of 2018 - Engineering
%
%Note: This function really handles more than 'braking', it determines the
%PodPhase, which is central to possible control behavior modes for the
%various control algorithms. We can actually move 'braking' code out of
%this function into it's own independednt (very consise) function because
%85% of this is really 'Pod Phase checking'
%
%ASSUME: 
%Imported KalmanStates are smoothened out (averaged) to eliminate noise and
%outliers... TODO
%'FlightTime' = 0 until pusher launches
%Pod at 3D origin when ready to launch, at rest
%
%global/shared variable types/sizes % TODO
%
%Input types: PodPhase (Int), BrakingModeStatus (Int), CommandPointCommands
%(Int Array), KalmanStates (10X1 double), IMUdata (?x? double),StripCount (Int), BrakingStripThreshold (Int),
%FlightTime (Int), BrakingTimeThreshold (Int), Fpb (Int), Feb (Int),
%TrackLength (Int), PlantStatus (Int Array),PusherReleaseDist (Int), Apush
%(Int), Mpod (Int), Aresist (Int), Amax (Int)
%Example Input:
%(2,8,[0],[1700.0 2.0 3.0; 4.0 5.0 6.0; 7.0 8.0 9.0],42,69,14444,340584,1400.0,14000.0,2000.0,[0, 0, 0, 0, 1],1600.0,20000,750.0,-1.0,-19.7)
%Start on pusher, triggered by pod position, change to coasting.
%
%Predetermined=static inputs (Some can become dynamic if necessary, but
%updated outside of this function. Others rely on average kinematic profile
%values and need to stay static to avoid unintentional triggers):
%BrakingStripThreshold (keep static),BrakingTimeThreshold (keep static),Fpb
%(initialize once externally,change internally if necessary, always output),Feb (keep static), TrackLength (keep
%static),PusherReleaseDist (keep static),PusherReleaseTime (keep
%static),Mpod (can be dynamic).
%
%Each iteration, update/store all dynamic inputs externally. Make sure the dynamic
%ones only initialized once where appropriate (ex. Fpb,Mpod,etc.)
%
%Output: 
%PodPhase (String),BrakingModeStatus (String),PrimaryBrakeCommand
%(String),FrontEBrakeCommand (String),BackEBrakeCommand
%(String),WheelsCommand (String),SkatesCommand (String), Fpb (Int)
%Example Output:
%(3,8,0,0,0,0,1,1400)
%
%Variable Descriptions: %specify units % TODO
%PodPhase (String) - current flight phase of pod.
%BrakingModeStatus (String) - current desired/signaled combination of braking, wheels,
%skates actuation.
%CommandPointCommands(Array) - Commands from the remote Command Point
%terminal including emergency stop signal.
%KalmanStates (2D Array) - Full set of current Pod Kalman States.
%StripCount (Int) - Number of detected retroreflective strips passed.
%BrakingStripThreshold (Int) - Predermined/calculated number of strips allowed to pass
%before BrakingPhase should trigger, late estimate.
%FlightTime (Int) - time elapsed, where t=0 at pusher launch start.
%BrakingTimeThreshold (Int) - Predetermined/calculated Flight time allowed to elapsed before BrakingPhase should trigger, late estimate 
%Fpb (Int) - Braking Force of Primary brakes in x-axis (negative val).
%Feb (Int) - Braking Force of all E-brakes in x-axis (negative val).
%TrackLength (Int) - Distance from front of pod (?) to end of track.
%PlantStatus (Array) - Current state (including currently changing state) of each control plant.
%PusherReleaseDist (Int) - Predetermined distance traveled by pod from start before
%release.
%PusherReleaseTime (Int) - Predetermined Flight Time elapsed when pusher
%releases pod.
%Mpod (Int) - Current Mass of entire Pod.
%Aresist (Int) - Average decceleration while coasting over a few time steps (from
%friction/contact/etc), calculate outside when 'Coasting Phase' AND brakes
%0, update (negative val).
%Amax (Int) - max negative decceleration allowed by SpaceX (negative val).

%DETERMINE BRAKINGMODESTATUS AND PODPHASE (EVENTUALLY): PodPhase and
%BrakingModeStatus input initialized as STANDBY in main script, before
%calling this function for the first time!!!

%Local Variables:
Ax = IMUglobaldata(1:1); %x-acceleration from IMU %update var % TODO
%Note: Not all math in conditionals assigned to variables beforehand to save unnesessary
%computation time

STANDBY = 0; %Both
%PodPhase
PREFLIGHT = 1;
PUSHER_PHASE = 2;
COASTING_PHASE = 3;
BRAKING_PHASE = 4;
%BrakingModeStatus
FULL_STOP = 5;
FULL_STOP_LESS = 6;
PRIMARY_STOP = 7;
BRAKING_READY = 8;
OFF = 9;

%PodPhase input is initialized as PREFLIGHT in main script!!!

%%%Standby: (Sitting at Start)
if PodPhase == STANDBY %only STANDBY braking possible
    BrakingModeStatus = STANDBY;

%PodPhase input is switched to PREFLIGHT in main script!!!

%%%PreFlight:
elseif PodPhase == PREFLIGHT %All possible braking modes cycled
    %PlantStatus = [primarybrakes BackE-brake FrontE-brakes wheels skates]
    if isequal(PlantStatus, [0, 0, 0, 1, 0]) %if STANDBY stable
        BrakingModeStatus = OFF;
    elseif isequal(PlantStatus,[0, 0, 0, 0, 1]) %if OFF stable
        BrakingModeStatus = BRAKING_READY;
    elseif isequal(PlantStatus,[0, 0, 0, 1, 0]) %if BRAKING_READY stable
        BrakingModeStatus = PRIMARY_STOP;
    elseif isequal(PlantStatus,[1, 0, 0, 1, 0]) %if PRIMARY_STOP stable
        BrakingModeStatus = FULL_STOP;
    elseif isequal(PlantStatus,[1, 1, 1, 1, 0]) %if FULL_STOP stable
        BrakingModeStatus = FULL_STOP_LESS;
    elseif isequal(PlantStatus,[0, 1, 1, 1, 0]) %if FULL_STOP_LESS stable
        BrakingModeStatus = OFF;
    else
        BrakingModeStatus = STANDBY;
    end
    
%PodPhase is switched to PUSHER_PHASE in main script!!!

%%%PusherPhase:
%(NEVER Engage Brakes)
elseif PodPhase == PUSHER_PHASE %Off only
    if KalmanStates(1)>PusherReleaseDist +10 %xpos %Kalman xpos based, with 10m margin
        BrakingModeStatus = OFF; %DO NOT CHANGE
        PodPhase = COASTING_PHASE; %DO NOT CHANGE
    elseif FlightTime>PusherReleaseTime %timer based on predetermined pusher accel and pusher dist %assumes predetermined constant accel
        BrakingModeStatus = OFF; %DO NOT CHANGE
        PodPhase = COASTING_PHASE; %DO NOT CHANGE   
    else
        BrakingModeStatus = OFF; %DO NOT CHANGE
        PodPhase = PUSHER_PHASE; %DO NOT CHANGE
    end
        
%FULL_STOP check/permanency and EmergencyMode check. This makes FULL_STOP
%mode permanent after entering COASTING_PHASE. (but can be activated
%anytime except during PUSHER_PHASE):
%Check if FULL_STOP already activated to keep it locked in:
elseif BrakingModeStatus == FULL_STOP
    BrakingModeStatus = FULL_STOP;
    if Ax<Amax %if Fullstop exceeds maximum decceleration allowed by SpaceX
        BrakingModeStatus = FULL_STOP_LESS; %Turn off Primary Brake
    end
elseif BrakingModeStatus == FULL_STOP_LESS
    BrakingModeStatus = FULL_STOP_LESS;
    
%Check if Emergency Stop Mode from CommandPoint (engage all brakes):
elseif CommandPointCommands(1) == 1 %update var % TODO
    BrakingModeStatus = FULL_STOP;
    
%Check if plant is mid-actuation. Don't switch modes if it is. Waits until
%plants are stable
elseif True
    for i = 5
        if ~(isequal(PlantStatus(i),0) || isequal(PlantStatus(i),1))
        end
    end


%%%CoastingPhase:
%check to start 'Braking Phase' (3 checks)
elseif PodPhase == COASTING_PHASE %FULL_STOP or temporary PRIMARY_STOP braking possible
    %Check Kalman (xpos Live Trajectory based)
    if -1/(2*Ax+Fpb/Mpod)*KalmanStates(4)^2 +5 > (TrackLength-KalmanStates(1)) %xvel,xpos %if absolute stopping distance with primarybrakes (constant decceleration) < x remaining (track length-distance travelled) %(trajectory based, so if primary brakes activate now, pod stops at 5m before end)
        BrakingModeStatus = PRIMARY_STOP;
        PodPhase = BRAKING_PHASE;
        
    %Check StripCount (rely on photoelectric, but trigger late)
    elseif StripCount >= BrakingStripThreshold %caclulate for input var
        BrakingModeStatus = PRIMARY_STOP;
        PodPhase = BRAKING_PHASE;
    
    %Check Timer (predetermined/rough, so very liberal estimate/last ditch check/least likely to trigger)
    elseif FlightTime > BrakingTimeThreshold %caclulate for input var TODO
        BrakingModeStatus = PRIMARY_STOP;
        PodPhase = BRAKING_PHASE;
        
%     elseif false %add conditional temporary primarybraking (based on pod vibrations/oscillations...) TODO
%         BrakingModeStatus = PRIMARY_STOP;
%         PodPhase = COASTING_PHASE; %not having this, lat control only
%         during coasting
        
    else %no brakes (and recover from temp braking)
        BrakingModeStatus = OFF;
        PodPhase = COASTING_PHASE;
    end
          

%%%Braking Phase: (Trust Kalman)
elseif PodPhase == BRAKING_PHASE %PRIMARY_STOP or FULL_STOP braking possible
   MinStoppingAx = -0.5/(TrackLength-KalmanStates(1))*KalmanStates(4)^2; %xpos %xvel %min accel required to stop the pod at the end at any given time %update var % TODO
   %Check if actual primary braking force higher/lower than expected. update Fpb
   if BrakingModeStatus == PRIMARY_STOP
       %Check if real pb value differs
       if abs(Ax-((Fpb/Mpod)+Aresist)*1.05) > (Fpb/Mpod)+Aresist  % margin of error (5%), so only triggers if significant difference, less jittery
           Fpb = (Ax-Aresist)*Mpod; %update true Fpb
       end  
       %Check if we can release pb for a bit
       if -1/(2*Ax)*KalmanStates(4)^2 +5 < (TrackLength-KalmanStates(1)) %if pb physically too high, BRAKING_READY!!!?
           BrakingModeStatus = BRAKING_READY;
           
      %Check if E-brakes needed (actual primary braking force lower than expected %Fpb)
       elseif Ax > MinStoppingAx %triggers as soon as we determine more deceleration is needed (will overshoot end with just pb)
           if -1/(2*(Feb+Fpb)/Mpod)*KalmanStates(4)^2 + 5 > (TrackLength-KalmanStates(1)) %xvel, xpos %FullStop abs stopping distance > Distance left %triggers as late as we can %margin of error (5 m for now, increase if less confident in Feb) %update var % TODO
               BrakingModeStatus = FULL_STOP; %if Feb higher than expected, doesnt matter, locked in anyway
           else
               BrakingModeStatus = PRIMARY_STOP;
           end
       else
           BrakingModeStatus = PRIMARY_STOP;
       end
   
   elseif BrakingModeStatus == BRAKING_READY
       if -1/(2*Ax+Fpb/Mpod)*KalmanStates(4)^2 +5 > (TrackLength-KalmanStates(1)) %xvel,xpos %if absolute stopping distance with primarybrakes (constant decceleration) < x remaining (track length-distance travelled) %(trajectory based, so if primary brakes activate now, pod stops at 5m before end)
           BrakingModeStatus = PRIMARY_STOP;
       end
   end
end

%SET REST OF OUTPUT COMMANDS:
if BrakingModeStatus == STANDBY %do we want primary brakes on while pod is sitting there?
    PrimaryBrakeCommand = 0;
    BackEBrakeCommand = 0;
    FrontEBrakeCommand = 0;
    WheelsCommand = 1;
    SkatesCommand = 0;
elseif BrakingModeStatus == FULL_STOP % assuming accurate Feb calculation
    PrimaryBrakeCommand = 1;
    BackEBrakeCommand = 1;
    FrontEBrakeCommand = 1;
    WheelsCommand = 1;
    SkatesCommand = 0;
elseif BrakingModeStatus == FULL_STOP_LESS  %in case FullStop exceeds 2g (would only be necessary if we underestimate braking power)
    PrimaryBrakeCommand = 0;
    BackEBrakeCommand = 1;
    FrontEBrakeCommand = 1;
    WheelsCommand = 1;
    SkatesCommand = 0;
elseif BrakingModeStatus == PRIMARY_STOP %make sure elsewhere wheels drop for (x) time eefore brakes applied
    PrimaryBrakeCommand = 1;
    BackEBrakeCommand = 0;
    FrontEBrakeCommand = 0;
    WheelsCommand = 1;
    SkatesCommand = 0;
elseif BrakingModeStatus == BRAKING_READY
    PrimaryBrakeCommand = 0;
    BackEBrakeCommand = 0;
    FrontEBrakeCommand = 0;
    WheelsCommand = 1;
    SkatesCommand = 0;
elseif BrakingModeStatus == OFF
    PrimaryBrakeCommand = 0;
    BackEBrakeCommand = 0;
    FrontEBrakeCommand = 0;
    WheelsCommand = 0;
    SkatesCommand = 1;
else %shouldn't ever happen
    PrimaryBrakeCommand = 0;
    BackEBrakeCommand = 0;
    FrontEBrakeCommand = 0;
    WheelsCommand = 0;
    SkatesCommand = 1;
end

output = {PodPhase,BrakingModeStatus,PrimaryBrakeCommand,FrontEBrakeCommand,BackEBrakeCommand,WheelsCommand,SkatesCommand,Fpb};
end