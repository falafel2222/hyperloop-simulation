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
        
        allowNoise
        noiseModifier
    end
    methods
        function pd = podData(mass,length, width, height, numSteps)
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
            pd.rotMatrix=eye(3);
            
            pd.allowNoise=false;
            pd.noiseModifier=0.05;
        end
        
        function localV = toLocal(self, globalV)
            localV = self.rotMatrix\globalV;
        end
        
        function globalV = toGlobal(self, localV)
            globalV = self.rotMatrix*localV;
        end
        
        function [] = updateTranslational(self,accel,n)
            self.transAcc(:,n) = accel;
            self.transVel(:,n) = self.transVel(:,n-1) + timestep*self.transAcc(:,n);
            self.transPos(:,n) = self.transPos(:,n-1) + timestep*self.transVel(:,n);            
        end
        
        function [] = updateRotational(self,rotAccel,n)
            self.rotAcc(:,n)=rotAccel;
            self.self.rotVel(:,n)=self.self.rotVel(:,n-1) + timestep*self.rotAccel(:,n);                        
        end
        
        function [] = updateQuaternions(self,n)
            omegaMatrix=[  0 self.rotVel(3,n) -self.rotVel(2,n) self.rotVel(1,n);...
                            -self.rotVel(3,n) 0 self.rotVel(1,n) self.rotVel(2,n);...
                            self.rotVel(2,n) -self.rotVel(1,n) 0 self.rotVel(3,n);...
                            -self.rotVel(1,n) -self.rotVel(2,n) -self.rotVel(3,n) 0;];
            
            qn=self.q(:,n-1)+0.5*omegaMatrix*self.q(:,n-1);
            normQuat=sqrt(sum(qn.^2));
            qn=qn./normQuat;
            
            self.q(:,n) = qn;
            self.updateRotationMatrix()
        end
        
        function [] = updateRotationMatrix(self,n)
            q0=self.q(4,n);
            q1=self.q(1,n);
            q2=self.q(2,n);
            q3=self.q(3,n);
                    
            self.rotMatrix=[1-2*q2^2-2*q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
                            2*(q1*q2+q0*q3) 1-2*q1^2-2*q3^2 2*(q2*q3-q0*q1);...
                            2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*q1^2-2*q2^2];
        end
        
        function [] = applyTorques(self,listOfForces,n)
            netTorque=[0,0,0];
            for force=listOfForces
               if ~force.isLocal
                   force.toLocal(self.rotMatrix);
               end
               if self.allowNoise
                   magForce=norm(force.components);
                   noise=magForce*(-self.noiseModifier+(2*self.noiseModifier)*rand());
                   force.components=force.components+noise;
               end
               torque=cross(force.location,force.components);
               netTorque=netTorque+torque;
            end
            self.rotAcc(:,n) = self.tensor \ netTorque';
        end
        
        function [] = applyForces(self,listOfForces,n)
            netForce=[0,0,0];
            for force=listOfForces
               if force.isLocal
                   force.toGlobal(self.rotMatrix);
               end
               netForce=netForce+force.components;
            end
            self.transAcc(:,n)=netForce/self.mass;
        end
    end
end

    