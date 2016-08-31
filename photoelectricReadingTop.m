function output = photoelectricReadingTop(sensorPosition, podState,nextStrips,toWall, tube,pod)

%pull out variables of pod state
rx=podState(1);
ry=podState(2);
rz=podState(3);
q1=podState(7);  
q2=podState(8);
q3=podState(9);
q0=podState(10);
%generate rotation matrix
Rot=[1-2*q2^2-2*q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
 2*(q1*q2+q0*q3) 1-2*q1^2-2*q3^2 2*(q2*q3-q0*q1);...
 2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*q1^2-2*q2^2;];

%find sensor positions in global frame
sx=(Rot(1,:)*sensorPosition + rx);
sy=(Rot(2,:)*sensorPosition + ry);
sz=(Rot(3,:)*sensorPosition + rz);
p=sensorPosition;

phi=pod.photoElectricTilt;
l=sin(pod.angleOfPESensitivity)./(cos(phi).*cos(pod.angleOfPESensitivity+phi));
v=toWall;
m=pi./(tube.stripWidth/2 + v.*l);
    
xA=v.*(tan(phi)+l);
xB=v.*(tan(phi)-l);

g=0.5*pod.peMax*(1+cos(m.*((v).*tan(phi)+sx-nextStrips)));

%PE reading
output=g.*(xA+sx+tube.stripWidth/2>nextStrips).*((xB+sx-tube.stripWidth/2)<nextStrips);
        