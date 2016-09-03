function [ output ] = PhaseAndBraking( PodPhase,BrakingModeStatus,CommandPointCommands,IMUglobaldata,KalmanStates,StripCount,BrakingStripThreshold,FlightTime,BrakingTimeThreshold,Fpb,Feb,TrackLength,PlantStatus,PusherReleaseDist,PusherReleaseTime,Mpod,Aresist,Amax,ReleaseDistMargin,PrimaryBrakeStoppingDistanceMargin,PBerrorMargin,CurrentStoppingDistanceMargin,FullBrakeSoppingDistanceMargin)
%OpenLoop
%SpaceX Hyperloop Pod Competition 2015-2016
%Control - Braking Algorithm
%Version 0.9
%Updated: 08/29/2016
%Chris Kotcherha - Harvey Mudd College - Class of 2018 - Engineering
%
%Note: This function really handles more than 'braking', it determines the
%'PodPhase', which is central to possible control behavior modes for the
%various control algorithms.
%
%ASSUME: 
%'FlightTime' = 0 until pusher launches
%Pod at 3D origin when ready to launch, at rest
%
%Input types: 
%PodPhase (Int), BrakingModeStatus (Int), CommandPointCommands (Int Array),
%KalmanStates (10X1 double), IMUdata (?x? double),StripCount (Int),
%BrakingStripThreshold (Int), FlightTime (Int), BrakingTimeThreshold (Int),
%Fpb (1X1 Double), Feb (1X1 Double), TrackLength (1X1 Double), PlantStatus
%(Int Array),PusherReleaseDist (1X1 Double), Mpod (1X1 Double), Aresist
%(1X1 Double), Amax (1x1 Double),ReleaseDistMargin (1x1 Double),
%PrimaryBrakeStoppingDistanceMargin (1x1 Double),PBerrorMargin (1x1
%Double),CurrentStoppingDistanceMargin (1x1
%Double),FullBrakeSoppingDistanceMargin (1x1 Double)
% 
%Example Input:
%
%
%Output: 
%PodPhase (String),BrakingModeStatus (String),PrimaryBrakeCommand
%(String),FrontEBrakeCommand (String),BackEBrakeCommand
%(String),WheelsCommand (String),SkatesCommand (String), Fpb (Int)
%Example Output:
%(3,8,0,0,0,0,1,1400)
%
%Variable Descriptions: %Metric (m, m/s m/s^2, N, ms)
%PodPhase (Int Array) - current flight phase of pod. 
%BrakingModeStatus (Int) - current desired/signaled combination of braking,
%wheels, skates actuation.
%CommandPointCommands(Int Array) - Commands from the remote Command Point
%terminal including emergency stop signal.
%KalmanStates (1X10 Double) - Full set of current Pod Kalman States.
%StripCount (Int) - Number of detected retroreflective strips passed.
%BrakingStripThreshold (Int) - Predermined/calculated number of strips
%allowed to pass before BrakingPhase should trigger, late estimate.
%FlightTime (Int) - time elapsed, where t=0 at pusher launch start.
%BrakingTimeThreshold (Int) - Predetermined/calculated Flight time allowed
%to elapsed before BrakingPhase should trigger, late estimate
%Fpb (1X1 Double) - Braking Force of Primary brakes in x-axis (negative val).
%Feb (1X1 Double) - Braking Force of all E-brakes in x-axis (negative val).
%TrackLength (1X1 Double) - Distance from front of pod (?) to end of track.
%PlantStatus (Int Array) - Current state (including currently changing
%state) of each control plant.
%PusherReleaseDist (1X1 Double) - Predetermined distance traveled by pod
%from start before release.
%PusherReleaseTime (Int) - Predetermined Flight Time elapsed when pusher
%releases pod.
%Mpod (1X1 Double) - Current Mass of entire Pod.
%Aresist (1X1 Double) - Average decceleration while coasting over a few
%time steps (from friction/contact/etc), calculate outside when 'Coasting
%Phase' AND brakes 0, update (negative val).
%Amax (1X1 Double) - max negative decceleration allowed by SpaceX (negative
%val).
%ReleaseDistMargin (1x1 Double) - How much farther after calculated pusher
%release distance pusher release can trigger.
%PrimaryBrakeStoppingDistanceMargin (1x1 Double) - How far sooner before
%calculated PB stopping distance point primary brakes will trigger.
%PBerrorMargin (1x1 Double) - Factor more or less that the real PB force
%can be than theortical PB braking force can be before a recalculation is
%triggered. (.05 = ±5%)
%CurrentStoppingDistanceMargin (1x1 Double) - How far sooner before track
%end calculated current stopping distance point needs to be for primary
%brakes to temporarily release.
%FullBrakeSoppingDistanceMargin (1x1 Double) - How far sooner before
%calculated PB+EB stopping distance needs to be for e-brakes to trigger.

%DETERMINE BRAKINGMODESTATUS AND PODPHASE (EVENTUALLY): PodPhase and
%BrakingModeStatus input initialized as STANDBY in main script, before
%calling this function for the first time!!!

%Local Variables (used often):
Ax = IMUglobaldata(3); %x-acceleration from IMU
xpos = KalmanStates(1);
xvel = KalmanStates(4);
%Note:some local variables initialized later to avoid unnecessary
%computatuon

%MODES:
STANDBY = 0; %Used for Both
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
    if xpos>PusherReleaseDist + ReleaseDistMargin %xpos %Kalman xpos based, with margin
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
if True
    for i = 5
        if ~(isequal(PlantStatus(i),0) || isequal(PlantStatus(i),1))
        end
    end
end

%%%CoastingPhase:
%check to start 'Braking Phase' (3 checks)
elseif PodPhase == COASTING_PHASE %FULL_STOP or temporary PRIMARY_STOP braking possible
    PrimaryBrakeStoppingDistance = -1/(2*(Ax+Fpb/Mpod))*xvel^2 + PrimaryBrakeStoppingDistanceMargin;
    TrackDistanceLeft = (TrackLength-xpos);
    %Check Kalman (xpos Live Trajectory based)
    
    if PrimaryBrakeStoppingDistance > TrackDistanceLeft %xvel,xpos %if absolute stopping distance with primarybrakes (constant decceleration) < x remaining (track length-distance travelled) %(trajectory based, so if primary brakes activate now, pod stops at 5m before end)
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
   PrimaryBrakeStoppingDistance = -1/(2*(Ax+Fpb/Mpod))*xvel^2 + PrimaryBrakeStoppingDistanceMargin;
   FullBrakeStopppingDistance = -1/(2*((Feb+Fpb)/Mpod))*xvel^2 + FullBrakeSoppingDistanceMargin;
   CurrentStoppingDistance = -1/(2*Ax)*xvel^2 + CurrentStoppingDistanceMargin;
   TrackDistanceLeft = (TrackLength-xpos);
   TheoreticalPBTotalForce = (Fpb/Mpod)+Aresist;
   MinStoppingAx = -0.5/TrackDistanceLeft*xvel^2; %xpos %xvel %min accel required to stop the pod at the end at any given time %update var % TODO
   %Check if actual primary braking force higher/lower than expected. update Fpb
   if BrakingModeStatus == PRIMARY_STOP
       %Check if real pb value differs
       if abs(Ax-(TheoreticalPBTotalForce)) > Fpb*PBerrorMargin  % margin of error (.05 = 5%), so only triggers if significant difference, less jittery
           Fpb = (Ax-Aresist)*Mpod; %update true Fpb
       end  
       %Check if we can release pb for a bit
       if CurrentStoppingDistance < TrackDistanceLeft %if pb physically too high, BRAKING_READY!!!?
           BrakingModeStatus = BRAKING_READY;
           
      %Check if E-brakes needed (actual primary braking force lower than expected %Fpb)
       elseif Ax > MinStoppingAx %triggers as soon as we determine more deceleration is needed (will overshoot end with just pb)
           if FullBrakeStopppingDistance > TrackDistanceLeft %xvel, xpos %FullStop abs stopping distance > Distance left %triggers as late as we can %margin of error (5 m for now, increase if less confident in Feb) %update var % TODO
               BrakingModeStatus = FULL_STOP; %if Feb higher than expected, doesnt matter, locked in anyway
           else
               BrakingModeStatus = PRIMARY_STOP;
           end
       else
           BrakingModeStatus = PRIMARY_STOP;
       end
   
   elseif BrakingModeStatus == BRAKING_READY
       if PrimaryBrakeStoppingDistance > TrackDistanceLeft %xvel,xpos %if absolute stopping distance with primarybrakes (constant decceleration) < x remaining (track length-distance travelled) %(trajectory based, so if primary brakes activate now, pod stops at 5m before end)
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