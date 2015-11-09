classdef podData
    properties
        transPos
        transVel
        transAcc

        rotPos
        rotVel
        rotAcc
        rotMatrix
        
        q
        mass
        tensor
        length
        width
        height

        netForce
        netTorque
        
        leftSkate
        rightSkate
        leftRailWheels
        rightRailWheels
        
        allowNoise
        noiseModifier
        
        timestep
        n
    end
    methods
        function pd = podData(mass,length, width, height, numSteps,timestep)
            pd.mass = mass;
            pd.length = length;
            pd.width = width;
            pd.height = height;
            pd.tensor=[   1.0/12*mass*(height^2 + width^2) 0 0; ...
                            0 1.0/12*mass*(height^2 + length^2) 0; ...
                            0 0 1.0/12*mass*(length^2 + width^2)];
            
            pd.transPos = zeros(3, numSteps);
            pd.transVel = zeros(3, numSteps);
            pd.transAcc = zeros(3, numSteps);

            pd.rotPos = zeros(3, numSteps);
            pd.rotVel = zeros(3, numSteps);
            pd.rotAcc = zeros(3, numSteps);

            pd.q=zeros(4, numSteps);
            pd.q(:,1)=[0;0;0; 1];
            pd.rotMatrix=eye(3);
            
            pd.netForce = [0 0 0]';
            pd.netTorque = [0 0 0]';
            
            pd.allowNoise=false;
            pd.noiseModifier=0.05;
            
            pd.timestep = timestep;
            pd.n = 2;
        end
        
        function localPos = toLocal(self, globalPos)
            localPos = self.rotMatrix\(globalPos-self.transPos(:,self.n-1));
        end
        
        function globalPos = toGlobal(self, localPos)
            globalPos = self.rotMatrix*localPos + self.transPos(:,self.n-1);
        end
        
        function self = updateTranslational(self,accel)
            self.transAcc(:,self.n) = accel;
            self.transVel(:,self.n) = self.transVel(:,self.n-1)...
                + self.timestep*self.transAcc(:,self.n);
            self.transPos(:,self.n) = self.transPos(:,self.n-1)...
                + self.timestep*self.transVel(:,self.n);
        end
        
        function self = updateRotational(self,rotAcc)
            self.rotAcc(:,self.n) = rotAcc;
            self.rotVel(:,self.n) = self.rotVel(:,self.n-1)...
                   + self.timestep*self.rotAcc(:,self.n);
        end
        
        function self = updateQuaternions(self)
            n = self.n;
            omegaMatrix=[  0 self.rotVel(3,n) -self.rotVel(2,n) self.rotVel(1,n);...
                            -self.rotVel(3,n) 0 self.rotVel(1,n) self.rotVel(2,n);...
                            self.rotVel(2,n) -self.rotVel(1,n) 0 self.rotVel(3,n);...
                            -self.rotVel(1,n) -self.rotVel(2,n) -self.rotVel(3,n) 0;];
            
            qn=self.q(:,self.n-1)+0.5*omegaMatrix*self.q(:,self.n-1);
            normQuat=sqrt(sum(qn.^2));
            qn=qn./normQuat;
            
            self.q(:,n) = qn;
            self = self.updateRotationMatrix();
        end
        
        function self = updateRotationMatrix(self)
            q0=self.q(4,self.n);
            q1=self.q(1,self.n);
            q2=self.q(2,self.n);
            q3=self.q(3,self.n);
                    
            self.rotMatrix=[1-2*q2^2-2*q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
                            2*(q1*q2+q0*q3) 1-2*q1^2-2*q3^2 2*(q2*q3-q0*q1);...
                            2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*q1^2-2*q2^2];
        end
        
        function self = applyForce(self,force)
           % Translational (in global)
           force = force.toGlobal(self.rotMatrix);
           self.netForce = self.netForce + force.components;

           % Rotational (in local)
           force = force.toLocal(self.rotMatrix);
           torque = cross(force.location,force.components);
           self.netTorque = self.netTorque + torque;
        end
        
        function self = update(self)
            self = self.updateRotational(self.tensor \ self.netTorque);
            self = self.updateTranslational(self.netForce / self.mass);
            self = self.updateQuaternions();
            self.netForce = [0;0;0];
            self.netTorque = [0;0;0];
            self.n = self.n + 1;
        end
    end
end

    