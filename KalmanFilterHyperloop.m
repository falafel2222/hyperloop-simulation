function [predictedState, predictedCovariance,localAcc] = KalmanFilterHyperloop( prevState, prevCovariance, IMUData, sensorData, execution, sensorUse,numberUsed,stripCounts,globals,pod,tube )

% globals = globalData();
% pod = podData();
% tube = tubeData();

% bottomDistancePositions=zeros(3,6);
% bottomDistancePositions=[0 0 pod.length/2 pod.length/2 -pod.length/2 -pod.length/2;pod.width/2 -pod.width/2 pod.width/2 -pod.width/2 pod.width/2 -pod.width/2;pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight pod.downSensorOffset-pod.skateHeight];
% downRailDistancePositions=[pod.length/2 pod.length/3 0 -pod.length/3 -pod.length/2;0 0 0 0 0;pod.downSensorOffset  pod.downSensorOffset pod.downSensorOffset pod.downSensorOffset pod.downSensorOffset];
% sideDistancePositions=[pod.length/2 pod.length/3 0 -pod.length/3 -pod.length/2;pod.sideSensorDistance pod.sideSensorDistance pod.sideSensorDistance pod.sideSensorDistance pod.sideSensorDistance;-pod.skateHeight/2 -pod.skateHeight/2 -pod.skateHeight/2 -pod.skateHeight/2 -pod.skateHeight/2];
% pitotPosition=[pod.length/2;0;pod.height*2/3];
% topPhotoElectricPositions=zeros(3,3);
% leftPhotoElectricPositions=zeros(3,3);
% rightPhotoElectricPositions=zeros(3,3);

% bottomDistanceDirections=[0 0 0 0 0 0; 0 0 0 0 0 0; -1 -1 -1 -1 -1 -1;];
% downRailDistanceDirections=[0 0 0 0 0 0; 0 0 0 0 0 0; -1 -1 -1 -1 -1 -1;];
% sideDistanceDirections=[0 0 0 0 0 0;  -1 -1 -1 -1 -1 -1; 0 0 0 0 0 0;];
% pitotDirection=[1;0;0];
% topPhotoElectricDirections=[0 0 0; 0 0 0; 1 1 1;];
% leftPhotoElectricDirections=[0 0 0; 1 1 1; 1 1 1;]/sqrt(2);
% rightPhotoElectricDirections=[0 0 0; -1 -1 -1; 1 1 1;]/sqrt(2);



thicknessOfRail = 0.0079502;
tubeCenterToTopOfRail=.4;
maxBrightness=1;
angleOfPESensitivity=10*pi/180;
trackHeight=-tube.railHeight;%0.72136;
railTopHeight=0;

stripWidth=2*0.0254;

%PE sensor formula assumes the reflectivities above, and
%that the photodiodes are sensitive in a cone of angle angleOfPESensitivity
%around the normal. It assumes that the signal received by each photodiode
%is, when considering the intersection of the tube and the cone traceed out
%by the diode's sensitivity (which is treated as a plane), that the signal is equal to the reflectivty of
%the tape times the area of the intersection that is tape, plus the
%reflectivity of the tube, times the area of the intersection that is tube.
%
%Note that this treats the strips as going the full 2pi radians around the
%tube and ignores color differences.
%
%This also assumes that the cone traced by the photodiode is small enough
%that it is impossible for the photodiode to recieve reflections from two
%strips at once, in any realistic position of the pod.
%
%It also plays fast and loose with the geometry of cones.
%
%This is a believable, but very ideal, set of assumptions. A more correct
%formula will be instituted after sensor tests of the photodiodes,
%including the use of two photodiodes and their difference as the signal,
%hence the placeholder variable distanceBetweenPE;

% p1 = sensorPositions(1,:)'; %Position of the sensor with respect to the center of mass of the pod
% p2 = sensorPositions(2,:)'; %Position of the sensor with respect to the center of mass of the pod
% p3 = sensorPositions(3,:)'; %Position of the sensor with respect to the center of mass of the pod
% %tck = thicknessOfRail/2;
% p4 = sensorPositions(4,:)'; %Position of the sensor with respect to the center of mass of the pod
% p5 = sensorPositions(4,:)'; %Position of the sensor with respect to the center of mass of the pod
% p6 = sensorPositions(4,:)'; %Position of the sensor with respect to the center of mass of the pod
% p7 = sensorPositions(4,:)'; %Position of the sensor with respect to the center of mass of the pod
% 
% 
% b4 = sensorDirections(4,:)'; %Ray indicating direction sensor points in local coordinates
% b5 = sensorDirections(5,:)'; %Ray indicating direction sensor points in local coordinates
% b6 = sensorDirections(6,:)'; %Ray indicating direction sensor points in local coordinates
% b7 = sensorDirections(7,:)'; %Ray indicating direction sensor points in local coordinates


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initializing state, kalman gain etc%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Currently using IMU as control to make the state prediction and so
%initializing that

%%%%%%%%%%%%%%%% ----------- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xkk=prevState; %This is the previous state and in the case of k=1 the initial state
uk=IMUData; %Control vector, currently IMU being used
Pkk=prevCovariance;

%
axL=uk(1);  %accelaration in x from IMU
ayL=uk(2);  %acceleration in y from IMU
azL=uk(3);  %acceleration in z from IMU
omegaX=uk(4);   %angular velocity in x from IMU
omegaY=uk(5);   %angular velocity in y from IMU
omegaZ=uk(6);   %angular velocity in z from IMU
%

%quaternions from the state
q1=xkk(7);  
q2=xkk(8);
q3=xkk(9);
q0=xkk(10);

%rotation matrix from the quaternions
Rot=[1-2*q2^2-2*q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
     2*(q1*q2+q0*q3) 1-2*q1^2-2*q3^2 2*(q2*q3-q0*q1);...
     2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*q1^2-2*q2^2;];


 %Fk matrix used for predicting the next state and the error in the
 %next state
%  Fk = [1 0 0 globals.kalmanTimestep 0 0 0 0 0 0;...
%        0 1 0 0 globals.kalmanTimestep 0 0 0 0 0;...
%        0 0 1 0 0 globals.kalmanTimestep 0 0 0 0;...
%        0 0 0 1 0 0 0 0 0 0;...
%        0 0 0 0 1 0 0 0 0 0;...
%        0 0 0 0 0 1 0 0 0 0;...
%        0 0 0 0 0 0 1 0 0 0;...
%        0 0 0 0 0 0 0 1 0 0;...
%        0 0 0 0 0 0 0 0 1 0;...
%        0 0 0 0 0 0 0 0 0 1;];

   Fk = [1 0 0 globals.kalmanTimestep 0 0 0 0 0 0;...
       0 1 0 0 globals.kalmanTimestep 0 0 0 0 0;...
       0 0 1 0 0 globals.kalmanTimestep 0 0 0 0;...
       0 0 0 1 0 0 globals.kalmanTimestep*(2*q2*ayL+2*q3*azL) globals.kalmanTimestep*(-4*q2*axL+2*q1*ayL+2*q0*azL) globals.kalmanTimestep*(-4*q3*axL-2*q0*ayL+2*q1*azL) globals.kalmanTimestep*(-2*q3*ayL+2*q2*azL);...
       0 0 0 0 1 0 globals.kalmanTimestep*(2*q2*axL-4*q1*ayL-2*q0*azL) globals.kalmanTimestep*(2*q1*axL+2*q3*azL) globals.kalmanTimestep*(2*q0*axL-4*q3*ayL+2*q2*azL) globals.kalmanTimestep*(2*q3*axL-2*q1*azL);...
       0 0 0 0 0 1 globals.kalmanTimestep*(2*q3*axL+2*q0*ayL-4*q1*azL) globals.kalmanTimestep*(-2*q0*axL+2*q3*ayL-4*q2*azL) globals.kalmanTimestep*(2*q1*axL+2*q2*ayL) globals.kalmanTimestep*(-2*q2*axL+2*q1*ayL);...
       0 0 0 0 0 0 1 globals.kalmanTimestep*omegaZ/2 -globals.kalmanTimestep*omegaY/2 globals.kalmanTimestep*omegaX/2;...
       0 0 0 0 0 0 -globals.kalmanTimestep*omegaZ/2 1 globals.kalmanTimestep*omegaX/2 globals.kalmanTimestep*omegaY/2;...
       0 0 0 0 0 0 globals.kalmanTimestep*omegaY/2 -globals.kalmanTimestep*omegaX/2 1 globals.kalmanTimestep*omegaZ/2;...
       0 0 0 0 0 0 -globals.kalmanTimestep*omegaX/2 -globals.kalmanTimestep*omegaY/2 -globals.kalmanTimestep*omegaZ/2 1;];
 



 %Bk matrix, multiplied to the control vector (currently IMU) in order
 %to also predict the next state of the pod
 Bk=[ 0 0 0 0 0 0;...
      0 0 0 0 0 0;...
      0 0 0 0 0 0;...
      Rot(1,1) Rot(1,2) Rot(1,3) 0 0 0;...
      Rot(2,1) Rot(2,2) Rot(2,3) 0 0 0;...
      Rot(3,1) Rot(3,2) Rot(3,3) 0 0 0;... %drop gravity term (constants=0 for this Jacobian)
      0 0 0 q0/2 -q3/2 q2/2;...
      0 0 0 q3/2 q0/2 -q1/2;...
      0 0 0 -q2/2 q1/2 q0/2;...
      0 0 0 -q1/2 -q2/2 -q3/2;]*globals.kalmanTimestep; 

%This is the portion for Wk and this is the same as patricks E80 code 
%for rn. We will be expirimentally determining this next semester%

%OLDuk=Control(:,k);
OmegaMatrix=[0 omegaZ -omegaY omegaX;...
            -omegaZ 0 omegaX omegaY;...
            omegaY -omegaX 0 omegaZ;...
            -omegaX -omegaY -omegaZ 0;];
% dqdt=(1/2).*(OmegaMatrix*xkk(7:10));
% dq1dt=dqdt(1);
% dq2dt=dqdt(2);
% dq3dt=dqdt(3);
% dq0dt=dqdt(4);
% domegadt=(NEXTuk(4:6)-OLDuk(4:6))./(2*globals.kalmanTimestep);
% domegaXdt=domegadt(1);
% domegaYdt=domegadt(2);
% domegaZdt=domegadt(3);


% wk=(0.5*globals.kalmanTimestep^2).*([(Rot*[axL;ayL;azL;])' ([1-4*q2*dq2dt-4*q3*dq3dt 2*(q1*dq2dt+q2*dq1dt-q0*dq3dt-q3*dq0dt) 2*(q1*dq3dt+q3*dq1dt+q0*dq2dt+q2*dq0dt);... %([1-4*q2*dq2dt-4*q3*dq3dt 2*(q1*dq2dt+q2*dq1dt+q0*dq3dt+q3*dq0dt) 2*(-q0*dq2dt-q2*dq0dt+q1*dq3dt+q3*dq1dt);%2*(-q0*dq3dt-q3*dq0dt+q1*dq2dt+q2*dq1dt) 1-4*q1*dq1dt-4*q3*dq3dt 2*(q2*dq3dt+q3*dq2dt+q0*dq1dt+q1*dq0dt); 2*(q1*dq3dt+q3*dq1dt+q0*dq2dt+q2*dq0dt) 2*(-q0*dq1dt-q1*dq0dt+q2*dq3dt+q3*dq2dt) 1-4*q2*dq2dt-4*q1*dq1dt;]*[axL;ayL;azL;]+Rot*([NEXTaxL-OLDaxL;NEXTayL-OLDayL;NEXTazL-OLDazL;]./(2*globals.kalmanTimestep)))'  
%                  2*(q1*dq2dt+q2*dq1dt+q0*dq3dt+q3*dq0dt) 1-4*q1*dq1dt-4*q3*dq3dt 2*(q2*dq3dt+q3*dq2dt-q0*dq1dt-q1*dq0dt);...
%                  2*(q1*dq3dt+q3*dq1dt-q0*dq2dt-q2*dq0dt) 2*(q2*dq3dt+q3*dq2dt+q0*dq1dt+q1*dq0dt) 1-4*q2*dq2dt-4*q1*dq1dt;]*[axL;ayL;azL;]+Rot*([NEXTaxL-OLDaxL;NEXTayL-OLDayL;NEXTazL-OLDazL;]./(2*globals.kalmanTimestep)))' ((1/2)*(OmegaMatrix*dqdt+...
%       [0 domegaZdt -domegaYdt domegaXdt;...
%       -domegaZdt 0 domegaXdt domegaYdt;...
%       domegaYdt -domegaXdt 0 domegaZdt;...
%       -domegaXdt -domegaYdt -domegaZdt 0;]*xkk(7:10)))'
%     ]').^2;
% Wk=(wk*wk')*(k~=1)+diag([wk])*(k==1);%diag(wk);
%Experimental determination bit ends%

Wk=zeros(10,10);


%this needs to be determined based on IMU error

 Qk = [diag((globals.IMUAccelSIMCovConst+globals.IMUAccelSIMCovLin.*abs(zeros(3,1)-globals.IMUAccelSIMCovZero)).*randn(3,1)) zeros(3,3);...
                zeros(3,3)  diag((globals.IMUGyroSIMCovConst+globals.IMUGyroSIMCovLin.*abs(zeros(3,1)-globals.IMUGyroSIMCovZero)).*randn(3,1));];








%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PREDICTION STEP

localAcc=(Rot*uk(1:3))-[0;0;globals.gravity;];
xkp1k=xkk+globals.kalmanTimestep*[  xkk(4:6);...
                       localAcc;...
                       (1/2).*(OmegaMatrix*xkk(7:10));];  %prediction step of the state 

Pkp1k=Fk*Pkk*Fk'+Bk*Qk*Bk'+Wk; %prediction step of the error, needs to be experimentally determined 


normQuat=sqrt(sum((xkp1k(7:10)).^2));
xkp1k(7:10)=xkp1k(7:10)./normQuat;


%%%%%%%%%%%%%%%%%%%----------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CORRECTIVE STEPS

% CORRECTIVE STEP 1, Bottom Distance sensors
%using minimum distance, not distance perpendicular to pod bottom.
if execution(1)==1 && numberUsed(1)~=0
    
    distDownUse=sensorUse(1:numberUsed(1),1);
    
    z1kp1 = sensorData((distDownUse),1);
    p1=pod.bottomDistancePositions(:,distDownUse);
    rz1=xkp1k(3);
    q11=xkp1k(7);  
    q12=xkp1k(8);
    q13=xkp1k(9);
    q10=xkp1k(10);
    Rot1=[1-2*q12^2-2*q13^2 2*(q11*q12-q10*q13) 2*(q11*q13+q10*q12);...
     2*(q11*q12+q10*q13) 1-2*q11^2-2*q13^2 2*(q12*q13-q10*q11);...
     2*(q11*q13-q10*q12) 2*(q12*q13+q10*q11) 1-2*q11^2-2*q12^2;];
    sz1=(Rot1(3,:)*p1+rz1); %+0.72136); why magic number?
    
    
    dd1dq1=[2*q13 2*q10 -4*q11]*p1;
    dd1dq2=[-2*q10 2*q13 -4*q12]*p1;
    dd1dq3=[2*q11 2*q12 0]*p1;
    dd1dq0=[-2*q12 2*q11 0]*p1;
    
    H1kp1=[zeros(numberUsed(1),1) zeros(numberUsed(1),1) ones(numberUsed(1),1) zeros(numberUsed(1),1) zeros(numberUsed(1),1) zeros(numberUsed(1),1) dd1dq1' dd1dq2' dd1dq3' dd1dq0'];%perpendicular to track
%     H1kp1=[0 0 ones(6,1)/Rot1(3,3) 0 0 0
%     (Rot1(3,3)*dd1dq1'-((sz1+trackHeight)*-4*q11))/((Rot1(3,3))^2) (Rot1(3,3)*dd1dq2'-((sz1+trackHeight)*-4*q12))/((Rot1(3,3))^2) (Rot1(3,3)*dd1dq3'-((sz1+trackHeight)*0)/((Rot1(3,3))^2) (Rot1(3,3)*dd1dq0'-((sz1+trackHeight)*0))/((Rot1(3,3))^2)]; %normal to pod bottom

%     S1kp1=diag([0.01,1000]);%XZScannerCovariance; %Experimentally determined
%     laserFactor = 100;
    S1kp1 = diag(globals.distDownCovConst(distDownUse)+globals.distDownCovLin(distDownUse).*abs(z1kp1-globals.distDownCovZero(distDownUse)));
%     disp('--------------------------------------')
%     disp(Pkp1k);
%     disp(H1kp1);
%     disp(H1kp1*Pkp1k*H1kp1');
%     disp(H1kp1*Pkp1k*H1kp1'+S1kp1);
    K1kp1=Pkp1k*H1kp1'/(H1kp1*Pkp1k*H1kp1'+S1kp1);


    h1kp1=sz1'-trackHeight; %perpendicular to track
    % h1kp1=(sz1+trackHeight)/Rot1(3,3); %normal to pod bottom
    x1kp1kp1=xkp1k+K1kp1*(z1kp1-h1kp1);
    P1kp1kp1=(eye(10,10)-K1kp1*H1kp1)*Pkp1k;
    
    normQuat1=sqrt(sum((x1kp1kp1(7:10)).^2));
    x1kp1kp1(7:10)=x1kp1kp1(7:10)./normQuat1;
else
    x1kp1kp1=xkp1k;
    P1kp1kp1=Pkp1k;
end



% CORRECTIVE STEP 2, downwards-pointing rail distance sensors
if execution(2)==1 && numberUsed(2)~=0
    
    distDownRailUse=sensorUse(1:numberUsed(2),2);
    
    z2kp1 = sensorData((distDownRailUse),2);
        
    p2=pod.downRailDistancePositions(:,distDownRailUse);
    rz2=x1kp1kp1(3);
    q21=x1kp1kp1(7);  
    q22=x1kp1kp1(8);
    q23=x1kp1kp1(9);
    q20=x1kp1kp1(10);
    Rot2=[1-2*q22^2-2*q23^2 2*(q21*q22-q20*q23) 2*(q21*q23+q20*q22);...
     2*(q21*q22+q20*q23) 1-2*q21^2-2*q23^2 2*(q22*q23-q20*q21);...
     2*(q21*q23-q20*q22) 2*(q22*q23+q20*q21) 1-2*q21^2-2*q22^2;];
    sz2=(Rot2(3,:)*p2+rz2);
    
    
    dd2dq1=[2*q23 2*q20 -4*q21]*p2;
    dd2dq2=[-2*q20 2*q23 -4*q22]*p2;
    dd2dq3=[2*q21 2*q22 0]*p2;
    dd2dq0=[-2*q22 2*q21 0]*p2;
     
        H2kp1=[zeros(numberUsed(2),1) zeros(numberUsed(2),1) ones(numberUsed(2),1) zeros(numberUsed(2),1) zeros(numberUsed(2),1) zeros(numberUsed(2),1) dd2dq1' dd2dq2' dd2dq3' dd2dq0'];%perpendicular to track
%     H2kp1=[0 0 ones(5,1)/Rot2(3,3) 0 0 0
%     (Rot2(3,3)*dd2dq1'-((sz2+railTopHeight)*-4*q21))/((Rot2(3,3))^2) (Rot2(3,3)*dd2dq2'-((sz2+railTopHeight)*-4*q22))/((Rot2(3,3))^2) (Rot2(3,3)*dd2dq3'-((sz2+railTopHeight)*0)/((Rot2(3,3))^2) (Rot2(3,3)*dd2dq0'-((sz2+railTopHeight)*0))/((Rot2(3,3))^2)]; %normal to pod bottom

    S2kp1=diag((globals.distDownRailCovConst(distDownRailUse)+globals.distDownRailCovLin(distDownRailUse).*abs(z2kp1-globals.distDownRailCovZero(distDownRailUse)))); %Experimentally determined
    K2kp1=P1kp1kp1*H2kp1'/(H2kp1*P1kp1kp1*H2kp1'+S2kp1);

    
    h2kp1=sz2'-railTopHeight; %perpendicular to track
    % h2kp1=(sz2+railTopHeight)/Rot2(3,3); %normal to pod bottom
      
    x2kp1kp1=x1kp1kp1+K2kp1*(z2kp1-h2kp1);
    P2kp1kp1=(eye(10,10)-K2kp1*H2kp1)*P1kp1kp1;
    
    normQuat2=sqrt(sum((x2kp1kp1(7:10)).^2));
    x2kp1kp1(7:10)=x2kp1kp1(7:10)./normQuat2;
else
    x2kp1kp1=x1kp1kp1;
    P2kp1kp1=P1kp1kp1;
end


% CORRECTIVE STEP 3, side distance sensors
if execution(3)==1 && numberUsed(3)~=0
    
    distSideUse=sensorUse(1:numberUsed(3),3);
    
    z3kp1 = sensorData((distSideUse),3);
    
    
    p3=pod.sideDistancePositions(:,distSideUse);
    ry3=x2kp1kp1(2);
    q31=x2kp1kp1(7);  
    q32=x2kp1kp1(8);
    q33=x2kp1kp1(9);
    q30=x2kp1kp1(10);
    Rot3=[1-2*q32^2-2*q33^2 2*(q31*q32-q30*q33) 2*(q31*q33+q30*q32);...
     2*(q31*q32+q30*q33) 1-2*q31^2-2*q33^2 2*(q32*q33-q30*q31);...
     2*(q31*q33-q30*q32) 2*(q32*q33+q30*q31) 1-2*q31^2-2*q32^2;];
    sy3=(Rot3(2,:)*p3+ry3);
    sq3=sqrt(Rot3(3,3)^2+Rot3(1,3)^2);
    m3=Rot3(1,1)*Rot3(3,3)-Rot3(1,3)*Rot3(3,1);
    
    
    dd3dq1=[2*q32 -4*q31 -2*q30]*p3;
    dd3dq2=[2*q31 0 2*q33]*p3;
    dd3dq3=[2*q30 -4*q33 2*q32]*p3;
    dd3dq0=[2*q33 0 -2*q31]*p3;
   
    
    H3kp1=[zeros(numberUsed(3),1) ones(numberUsed(3),1) zeros(numberUsed(3),1) zeros(numberUsed(3),1) zeros(numberUsed(3),1) zeros(numberUsed(3),1) dd3dq1' dd3dq2' dd3dq3' dd3dq0'];
%     H3kp1=[0 0 ones(5,1)/Rot3(2,2) 0 0 0 (Rot3(2,2)*dd3dq1'-((sy3-(thicknessOfRail/2))*-4*q31))/((Rot3(2,2))^2) (Rot3(2,2)*dd3dq2'-((sy3-(thicknessOfRail/2))*0))/((Rot3(2,2))^2) (Rot3(2,2)*dd3dq3'-((sy3-(thicknessOfRail/2))*-4*q33))/((Rot3(2,2))^2) (Rot3(2,2)*dd3dq0'-((sy3-(thicknessOfRail/2))*0))/((Rot3(2,2))^2)]; %normal to pod side

    S3kp1=diag(globals.distSideCovConst(distSideUse)+globals.distSideCovLin(distSideUse).*abs(z3kp1-globals.distSideCovZero(distSideUse))); %Experimentally determined
    K3kp1=P2kp1kp1*H3kp1'/(H3kp1*P2kp1kp1*H3kp1'+S3kp1);

    
    h3kp1=sy3'-(thicknessOfRail/2);%perpendicular to rail side
    % h3kp1=(sy3-(thicknessOfRail/2))/Rot3(2,2); %normal to pod side
        
    x3kp1kp1=x2kp1kp1+K3kp1*(z3kp1-h3kp1);
    P3kp1kp1=(eye(10,10)-K3kp1*H3kp1)*P2kp1kp1;
    
    normQuat3=sqrt(sum((x3kp1kp1(7:10)).^2));
    x3kp1kp1(7:10)=x3kp1kp1(7:10)./normQuat3;
else
    x3kp1kp1=x2kp1kp1;
    P3kp1kp1=P2kp1kp1;
end


% CORRECTIVE STEP 4, PITOT TUBE
if execution(4)==1 && numberUsed(4)~=0
    
    z4kp1 = sensorData(1,4);
    
    p4=pod.pitotPosition;
    b4=pod.pitotDirection;
    
    vPod4 = x3kp1kp1(4:6);
    q41=x3kp1kp1(7);  
    q42=x3kp1kp1(8);
    q43=x3kp1kp1(9);
    q40=x3kp1kp1(10);
    Rot4=[1-2*q42^2-2*q43^2 2*(q41*q42-q40*q43) 2*(q41*q43+q40*q42);...
     2*(q41*q42+q40*q43) 1-2*q41^2-2*q43^2 2*(q42*q43-q40*q41);...
     2*(q41*q43-q40*q42) 2*(q42*q43+q40*q41) 1-2*q41^2-2*q42^2;];
 
    dRot4dq1=[0 2*q42 2*q43;...
     2*q42 -4*q41 -2*q40;...
     2*q43 2*q40 -4*q41;];
 
    dRot4dq2=[-4*q42 2*q41 2*q40;...
     2*q41 0 2*q43;...
     -2*q40 2*q43 -4*q42;];
 
    dRot4dq3=[-4*q43 -2*q40 2*q41;...
     2*q40 -4*q43 2*q42;...
     2*q41 2*q42 0;];
 
    dRot4dq0=[0 -2*q43 2*q42;...
     2*q43 0 -2*q41;...
     -2*q42 2*q41 0;];
    
    
    ddP4dq1=globals.airDensity*(dot(vPod4,(Rot4*b4)))*(dot(vPod4,(dRot4dq1*b4)));
    ddP4dq2=globals.airDensity*(dot(vPod4,(Rot4*b4)))*(dot(vPod4,(dRot4dq2*b4)));
    ddP4dq3=globals.airDensity*(dot(vPod4,(Rot4*b4)))*(dot(vPod4,(dRot4dq3*b4)));
    ddP4dq0=globals.airDensity*(dot(vPod4,(Rot4*b4)))*(dot(vPod4,(dRot4dq0*b4)));
    
    ddP4dvx = globals.airDensity*(dot(vPod4,(Rot4*b4)))*(dot((Rot4*b4),[1 0 0]'));
    ddP4dvy = globals.airDensity*(dot(vPod4,(Rot4*b4)))*(dot((Rot4*b4),[0 1 0]'));
    ddP4dvz = globals.airDensity*(dot(vPod4,(Rot4*b4)))*(dot((Rot4*b4),[0 0 1]'));
    
    
    H4kp1=[0 0 0 ddP4dvx ddP4dvy ddP4dvz ddP4dq1 ddP4dq2 ddP4dq3 ddP4dq0];
    S4kp1=diag(globals.pitotCovConst+globals.pitotCovLin.*abs(z4kp1-globals.pitotCovZero)); %Experimentally determined
    K4kp1=P3kp1kp1*H4kp1'/(H4kp1*P3kp1kp1*H4kp1'+S4kp1);

    
    h4kp1=0.5*globals.airDensity*(dot(vPod4,b4))^2;
        
    x4kp1kp1=x3kp1kp1+K4kp1*(z4kp1-h4kp1);
    P4kp1kp1=(eye(10,10)-K4kp1*H4kp1)*P3kp1kp1;
    
    normQuat4=sqrt(sum((x4kp1kp1(7:10)).^2));
    x4kp1kp1(7:10)=x4kp1kp1(7:10)./normQuat4;
else
    x4kp1kp1=x3kp1kp1;
    P4kp1kp1=P3kp1kp1;
end



% CORRECTIVE STEP 5, 12 o'clock Photoelectric
if execution(5)==1 && numberUsed(5)~=0
    % getting sensor data
    
    
    peTopUse=sensorUse(1:numberUsed(5),5);
    
    z5kp1 = sensorData((peTopUse),5);
    
    p5=pod.topPhotoelectricPosition(peTopUse);
    b5=pod.topPhotoelectricDirection(peTopUse);
    
    % Getting terms from the previous state prediction to use in calculation
    rx5=x4kp1kp1(1);
    ry5=x4kp1kp1(2);
    rz5=x4kp1kp1(3);
    q51=x4kp1kp1(7);  
    q52=x4kp1kp1(8);
    q53=x4kp1kp1(9);
    q50=x4kp1kp1(10);
    %Calculating the rotation matrix
    Rot5=[1-2*q52^2-2*q53^2 2*(q51*q52-q50*q53) 2*(q51*q53+q50*q52);...
     2*(q51*q52+q50*q53) 1-2*q51^2-2*q53^2 2*(q52*q53-q50*q51);...
     2*(q51*q53-q50*q52) 2*(q52*q53+q50*q51) 1-2*q51^2-2*q52^2;];
    sx5=(Rot5(1,:)*p5 + rx5)';
    sy5=(Rot5(2,:)*p5 + ry5)';
    sz5=(Rot5(3,:)*p5 + rz5)';
    
    
    
    nextStrips5=tube.stripDistances(stripCounts(1:3)+1);
    
    %derivatives of the rotation matrix w.r.t. each quaternion to be used to calculate the jacobian
    dRot5dq1=[0 2*q52 2*q53;...
     2*q52 -4*q51 -2*q50;...
     2*q53 2*q50 -4*q51;];
 
    dRot5dq2=[-4*q52 2*q51 2*q50;...
     2*q51 0 2*q53;...
     -2*q50 2*q53 -4*q52;];
 
    dRot5dq3=[-4*q53 -2*q50 2*q51;...
     2*q50 -4*q53 2*q52;...
     2*q51 2*q52 0;];
 
    dRot5dq0=[0 -2*q53 2*q52;...
     2*q53 0 -2*q51;...
     -2*q52 2*q51 0;];
     
    phi5=pod.photoElectricTilt; %could add pitch here
    l5=sin(pod.angleOfPESensitivity)./(cos(phi5).*cos(pod.angleOfPESensitivity+phi5));
    v5=(pod.peToWall(1:3)); %could be updated for tube curvature
    m5=pi./(tube.stripWidth/2 + v5.*l5);
    
    xA5=v5.*(tan(phi5)+l5);
    xB5=v5.*(tan(phi5)-l5);
    %Finally calculating the actual terms for the Jacobian
    dg5drx=-0.5*pod.peMax.*sin(m5.*(v5.*tan(phi5)+sx5-nextStrips5)).*m5;
    %dg5dry=-0.5*pod.peMax.*sin(m5.*(v5.*tan(phi5)+sx5-nextStrips5)).*((-pi
    %*l5.*dv5dy/((tube.stripWidth.^2+v5.*l5).^2))*(v5.*tan(phi5)+sx5-nextStrips5) + m.*(v5.*(sec(phi5)).^2*dphi5dy + dv5dy.*tan(phi5)));
    
    dg5dq1=-0.5*pod.peMax.*sin(m5.*(v5.*tan(phi5)+sx5-nextStrips5)).*m5.*(dRot5dq1(1,:)*p5)';
    dg5dq2=-0.5*pod.peMax.*sin(m5.*(v5.*tan(phi5)+sx5-nextStrips5)).*m5.*(dRot5dq2(1,:)*p5)';
    dg5dq3=-0.5*pod.peMax.*sin(m5.*(v5.*tan(phi5)+sx5-nextStrips5)).*m5.*(dRot5dq3(1,:)*p5)';
    dg5dq0=-0.5*pod.peMax.*sin(m5.*(v5.*tan(phi5)+sx5-nextStrips5)).*m5.*(dRot5dq0(1,:)*p5)';
    
    
    H5kp1=[dg5drx zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) dg5dq1 dg5dq2 dg5dq3 dg5dq0].*(((xA5+sx5+tube.stripWidth/2>nextStrips5)*((xB5+sx5-tube.stripWidth/2)<nextStrips5))*ones(1,10));
    S5kp1=VerticalPECovariance; %Experimentally determined
    K5kp1=P4kp1kp1*H5kp1'/(H5kp1*P4kp1kp1*H5kp1'+S5kp1); %Kalman Gain

    g5=0.5*pod.peMax*(1+cos(m5.*((v5).*tan(phi5)+sx5-nextStrips5)));
    
    h5kp1=g5.*(xA5+sx5+tube.stripWidth/2>nextStrips5).*((xB5+sx5-tube.stripWidth/2)<nextStrips5);
    % The next step compares the data from the sensors with the predicted state and alters
    %the state prediction by a factor determined by the Kalman Gain
    x5kp1kp1=x4kp1kp1+K5kp1*(z5kp1-h5kp1);
    P5kp1kp1=(eye(10,10)-K5kp1*H5kp1)*P4kp1kp1;
    
    normQuat5=sqrt(sum((x5kp1kp1(7:10)).^2));
    x5kp1kp1(7:10)=x5kp1kp1(7:10)./normQuat5;
else
    x5kp1kp1=x4kp1kp1;
    P5kp1kp1=P4kp1kp1;
end




% CORRECTIVE STEP 6, 10:30 Photoelectric (positive y)
if execution(6)==1 && numberUsed(6)~=0
    % getting sensor data
    
    peLeftUse=sensorUse(1:numberUsed(6),6);
    
    
    p6=pod.leftPhotoelectricPosition(peLeftUse);
    b6=pod.leftPhotoelectricDirection(peLeftUse);
    z6kp1 = sensorData(peLeftUse,6);
    
    % Getting terms from the previous state prediction to use in calculations
    rx6=x5kp1kp1(1);
    ry6=x5kp1kp1(2);
    rz6=x5kp1kp1(3);
    q61=x5kp1kp1(7);  
    q62=x5kp1kp1(8);
    q63=x5kp1kp1(9);
    q60=x5kp1kp1(10);
    %Calculating the rotation matrix
    Rot6=[1-2*q62^2-2*q63^2 2*(q61*q62-q60*q63) 2*(q61*q63+q60*q62);...
     2*(q61*q62+q60*q63) 1-2*q61^2-2*q63^2 2*(q62*q63-q60*q61);...
     2*(q61*q63-q60*q62) 2*(q62*q63+q60*q61) 1-2*q61^2-2*q62^2;];
    sx6=(Rot6(1,:)*p6 + rx6)';
    sy6=(Rot6(2,:)*p6 + ry6)';
    sz6=(Rot6(3,:)*p6 + rz6)';
    
    
    nextStrips6=tube.stripDistances(stripCounts(4:6)+1);
    
    %derivatives of the rotation matrix w.r.t. each quaternion to be used to calculate the jacobian
    dRot6dq1=[0 2*q62 2*q63;...
     2*q62 -4*q61 -2*q60;...
     2*q63 2*q60 -4*q61;];
 
    dRot6dq2=[-4*q62 2*q61 2*q60;...
     2*q61 0 2*q63;...
     -2*q60 2*q63 -4*q62;];
 
    dRot6dq3=[-4*q63 -2*q60 2*q61;...
     2*q60 -4*q63 2*q62;...
     2*q61 2*q62 0;];
 
    dRot6dq0=[0 -2*q63 2*q62;...
     2*q63 0 -2*q61;...
     -2*q62 2*q61 0;];
     
    phi6=pod.photoElectricTilt; %could add yaw here (also pitch)
    l6=sin(pod.angleOfPESensitivity)./(cos(phi6).*cos(pod.angleOfPESensitivity+phi6));
    v6=(pod.peToWall(4:6)).^2+(sy6-p6(2,:)).^2 -sqrt(2).*(sy6-p6(2,:)).*pod.peToWall(4:6); 
    m6=pi./(tube.stripWidth/2 + v6.*l6);
    
    xA6=v6.*(tan(phi6)+l6);
    xB6=v6.*(tan(phi6)-l6);
    %Finally calculating the actual terms for the Jacobian
    dg6drx=-0.5*pod.peMax.*sin(m6.*(v6.*tan(phi6)+sx6-nextStrips6)).*m6;
    dg6dry=-0.5*pod.peMax.*sin(m6.*(v6.*tan(phi6)+sx6-nextStrips6)).*((-pi*l6.*(2*(sy6-p6(2,:))-sqrt(2).*pod.peToWall(4:6))/((tube.stripWidth.^2+v6.*l6).^2)).*(v6.*tan(phi6)+sx6-nextStrips6) + m6.*((2*(sy6-p6(2,:))-sqrt(2).*pod.peToWall(4:6)).*tan(phi6)));
    
    dg6dq1=-0.5*pod.peMax.*sin(m6.*(v6.*tan(phi6)+sx6-nextStrips6)).*(m6.*((dRot6dq1(1,:)*p6)'+ tan(phi6).* (2*(sy6-p6(2,:))*(dRot6dq1(2,:)*p6)'-sqrt(2)*pod.peToWall(4:6).*(dRot6dq1(2,:)*p6)')) + ((-pi.*l5.*(2*(sy6-p6(2,:))*(dRot6dq1(2,:)*p6)'-sqrt(2)*pod.peToWall(4:6).*(dRot6dq1(2,:)*p6)'))./((tube.stripWidth/2 + v6.*l6).^2)).*(v6.*tan(phi6)+sx6-nextStrips6));
    dg6dq2=-0.5*pod.peMax.*sin(m6.*(v6.*tan(phi6)+sx6-nextStrips6)).*(m6.*((dRot6dq2(1,:)*p6)'+ tan(phi6).* (2*(sy6-p6(2,:))*(dRot6dq2(2,:)*p6)'-sqrt(2)*pod.peToWall(4:6).*(dRot6dq2(2,:)*p6)')) + ((-pi.*l5.*(2*(sy6-p6(2,:))*(dRot6dq2(2,:)*p6)'-sqrt(2)*pod.peToWall(4:6).*(dRot6dq2(2,:)*p6)'))./((tube.stripWidth/2 + v6.*l6).^2)).*(v6.*tan(phi6)+sx6-nextStrips6));
    dg6dq3=-0.5*pod.peMax.*sin(m6.*(v6.*tan(phi6)+sx6-nextStrips6)).*(m6.*((dRot6dq3(1,:)*p6)'+ tan(phi6).* (2*(sy6-p6(2,:))*(dRot6dq3(2,:)*p6)'-sqrt(2)*pod.peToWall(4:6).*(dRot6dq3(2,:)*p6)')) + ((-pi.*l5.*(2*(sy6-p6(2,:))*(dRot6dq3(2,:)*p6)'-sqrt(2)*pod.peToWall(4:6).*(dRot6dq3(2,:)*p6)'))./((tube.stripWidth/2 + v6.*l6).^2)).*(v6.*tan(phi6)+sx6-nextStrips6));
    dg6dq0=-0.5*pod.peMax.*sin(m6.*(v6.*tan(phi6)+sx6-nextStrips6)).*(m6.*((dRot6dq0(1,:)*p6)'+ tan(phi6).* (2*(sy6-p6(2,:))*(dRot6dq0(2,:)*p6)'-sqrt(2)*pod.peToWall(4:6).*(dRot6dq0(2,:)*p6)')) + ((-pi.*l5.*(2*(sy6-p6(2,:))*(dRot6dq0(2,:)*p6)'-sqrt(2)*pod.peToWall(4:6).*(dRot6dq0(2,:)*p6)'))./((tube.stripWidth/2 + v6.*l6).^2)).*(v6.*tan(phi6)+sx6-nextStrips6));
    
    
    H6kp1=[dg6drx dg6dry zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) dg6dq1 dg6dq2 dg6dq3 dg6dq0].*(((xA6+sx6+tube.stripWidth/2>nextStrips6)*((xB6+sx6-tube.stripWidth/2)<nextStrips6))*ones(1,10))
    S6kp1=VerticalPECovariance; %Experimentally determined
    K6kp1=P5kp1kp1*H6kp1'/(H6kp1*P5kp1kp1*H6kp1'+S6kp1); %Kalman Gain

    g6=0.5*pod.peMax*(1+cos(m6.*((v6).*tan(phi6)+sx6-nextStrips6)));
    
    h6kp1=g6.*(xA6+sx6+tube.stripWidth/2>nextStrips6).*((xB6+sx6-tube.stripWidth/2)<nextStrips6);
    % The next step compares the data from the sensors with the predicted state and alters
    %the state prediction by a factor determined by the Kalman Gain        
    x6kp1kp1=x5kp1kp1+K6kp1*(z6kp1-h6kp1);
    P6kp1kp1=(eye(10,10)-K6kp1*H6kp1)*P5kp1kp1;
    
    normQuat6=sqrt(sum((x6kp1kp1(7:10)).^2));
    x6kp1kp1(7:10)=x6kp1kp1(7:10)./normQuat6;
else
    x6kp1kp1=x5kp1kp1;
    P6kp1kp1=P5kp1kp1;
end



% CORRECTIVE STEP 7, 1:30 Photoelectric (negative y)
if execution(7)==1 && numberUsed(7)~=0
    % getting sensor data
    peRightUse=sensorUse(1:numberUsed(7),7);
    
    
    p7=pod.rightPhotoelectricPosition(peRightUse);
    b7=pod.rightPhotoelectricDirection(peRightUse);
    z7kp1 = sensorData(peLeftUse,7);
    
    % Getting terms from the previous state prediction to use in calculations
    rx7=x6kp1kp1(1);
    ry7=x6kp1kp1(2);
    rz7=x6kp1kp1(3);
    q71=x6kp1kp1(7);  
    q72=x6kp1kp1(8);
    q73=x6kp1kp1(9);
    q70=x6kp1kp1(10);
    %Calculating the rotation matrix
    Rot7=[1-2*q72^2-2*q73^2 2*(q71*q72-q70*q73) 2*(q71*q73+q70*q72);...
     2*(q71*q72+q70*q73) 1-2*q71^2-2*q73^2 2*(q72*q73-q70*q71);...
     2*(q71*q73-q70*q72) 2*(q72*q73+q70*q71) 1-2*q71^2-2*q72^2;];
    sx7=(Rot7(1,:)*p7 + rx7);
    sy7=(Rot7(2,:)*p7 + ry7);
    sz7=(Rot7(3,:)*p7 + rz7);
    
    nextStrips7=tube.stripDistances(stripCounts(7:9)+1);
    
    %derivatives of the rotation matrix w.r.t. each quaternion to be used to calculate the jacobian
    dRot7dq1=[0 2*q72 2*q73;...
     2*q72 -4*q71 -2*q70;...
     2*q73 2*q70 -4*q71;];
 
    dRot7dq2=[-4*q72 2*q71 2*q70;...
     2*q71 0 2*q73;...
     -2*q70 2*q73 -4*q72;];
 
    dRot7dq3=[-4*q73 -2*q70 2*q71;...
     2*q70 -4*q73 2*q72;...
     2*q71 2*q72 0;];
 
    dRot7dq0=[0 -2*q73 2*q72;...
     2*q73 0 -2*q71;...
     -2*q72 2*q71 0;];
     
    phi7=pod.photoElectricTilt; %could add yaw here (also pitch)
    l7=sin(pod.angleOfPESensitivity)./(cos(phi7).*cos(pod.angleOfPESensitivity+phi7));
    v7=(pod.peToWall(7:9)).^2+(sy7-p7(2,:)).^2 -sqrt(2).*(sy7-p7(2,:)).*pod.peToWall(7:9); 
    m7=pi./(tube.stripWidth/2 + v7.*l7);
    
    xA7=v7.*(tan(phi7)+l7);
    xB7=v7.*(tan(phi7)-l7);
    %Finally calculating the actual terms for the Jacobian
    dg7drx=-0.5*pod.peMax.*sin(m7.*(v7.*tan(phi7)+sx7-nextStrips7)).*m7;
    dg7dry=-0.5*pod.peMax.*sin(m7.*(v7.*tan(phi7)+sx7-nextStrips7)).*((-pi*l7.*(2*(sy7-p7(2,:))-sqrt(2).*pod.peToWall(7:9))/((tube.stripWidth.^2+v7.*l7).^2)).*(v7.*tan(phi7)+sx7-nextStrips7) + m7.*((2*(sy7-p7(2,:))-sqrt(2).*pod.peToWall(7:9)).*tan(phi7)));
    
    dg7dq1=-0.5*pod.peMax.*sin(m7.*(v7.*tan(phi7)+sx7-nextStrips7)).*(m7.*((dRot7dq1(1,:)*p7)'+ tan(phi7).* (2*(sy7-p7(2,:))*(dRot7dq1(2,:)*p7)'-sqrt(2)*pod.peToWall(7:9).*(dRot7dq1(2,:)*p7)')) + ((-pi.*l5.*(2*(sy7-p7(2,:))*(dRot7dq1(2,:)*p7)'-sqrt(2)*pod.peToWall(7:9).*(dRot7dq1(2,:)*p7)'))./((tube.stripWidth/2 + v7.*l7).^2)).*(v7.*tan(phi7)+sx7-nextStrips7));
    dg7dq2=-0.5*pod.peMax.*sin(m7.*(v7.*tan(phi7)+sx7-nextStrips7)).*(m7.*((dRot7dq2(1,:)*p7)'+ tan(phi7).* (2*(sy7-p7(2,:))*(dRot7dq2(2,:)*p7)'-sqrt(2)*pod.peToWall(7:9).*(dRot7dq2(2,:)*p7)')) + ((-pi.*l5.*(2*(sy7-p7(2,:))*(dRot7dq2(2,:)*p7)'-sqrt(2)*pod.peToWall(7:9).*(dRot7dq2(2,:)*p7)'))./((tube.stripWidth/2 + v7.*l7).^2)).*(v7.*tan(phi7)+sx7-nextStrips7));
    dg7dq3=-0.5*pod.peMax.*sin(m7.*(v7.*tan(phi7)+sx7-nextStrips7)).*(m7.*((dRot7dq3(1,:)*p7)'+ tan(phi7).* (2*(sy7-p7(2,:))*(dRot7dq3(2,:)*p7)'-sqrt(2)*pod.peToWall(7:9).*(dRot7dq3(2,:)*p7)')) + ((-pi.*l5.*(2*(sy7-p7(2,:))*(dRot7dq3(2,:)*p7)'-sqrt(2)*pod.peToWall(7:9).*(dRot7dq3(2,:)*p7)'))./((tube.stripWidth/2 + v7.*l7).^2)).*(v7.*tan(phi7)+sx7-nextStrips7));
    dg7dq0=-0.5*pod.peMax.*sin(m7.*(v7.*tan(phi7)+sx7-nextStrips7)).*(m7.*((dRot7dq0(1,:)*p7)'+ tan(phi7).* (2*(sy7-p7(2,:))*(dRot7dq0(2,:)*p7)'-sqrt(2)*pod.peToWall(7:9).*(dRot7dq0(2,:)*p7)')) + ((-pi.*l5.*(2*(sy7-p7(2,:))*(dRot7dq0(2,:)*p7)'-sqrt(2)*pod.peToWall(7:9).*(dRot7dq0(2,:)*p7)'))./((tube.stripWidth/2 + v7.*l7).^2)).*(v7.*tan(phi7)+sx7-nextStrips7));
    
    
    H7kp1=[dg7drx dg7dry zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) dg7dq1 dg7dq2 dg7dq3 dg7dq0].*(((xA7+sx7+tube.stripWidth/2>nextStrips7)*((xB7+sx7-tube.stripWidth/2)<nextStrips7))*ones(1,10));
    S7kp1=VerticalPECovariance; %Experimentally determined
    K7kp1=P6kp1kp1*H7kp1'/(H7kp1*P6kp1kp1*H7kp1'+S7kp1); %Kalman Gain

    g7=0.5*pod.peMax*(1+cos(m7.*((v7).*tan(phi7)+sx7-nextStrips7)));
    
    h7kp1=g7.*(xA7+sx7+tube.stripWidth/2>nextStrips7).*((xB7+sx7-tube.stripWidth/2)<nextStrips7);
    % The next step compares the data from the sensors with the predicted state and alters
    %the state prediction by a factor determined by the Kalman Gain 
    x7kp1kp1=x6kp1kp1+K7kp1*(z7kp1-h7kp1);
    P7kp1kp1=(eye(10,10)-K7kp1*H7kp1)*P6kp1kp1;
    
    normQuat7=sqrt(sum((x7kp1kp1(7:10)).^2));
    x7kp1kp1(7:10)=x7kp1kp1(7:10)./normQuat7;
else
    x7kp1kp1=x6kp1kp1;
    P7kp1kp1=P6kp1kp1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

predictedState = x7kp1kp1;
predictedCovariance= P7kp1kp1;
    
    
    
    
    
    
    
    

