classdef podData
    properties
        transPos
        transVel
        transAcc

        rotPos
        rotVel
        rotAcc

        q
        mass
        length
        width
        height
    end
    methods
        function pd = podData(mass,length, width,height, numSteps)
            pd.mass = mass;
            pd.length = length;
            pd.width = width;
            pd.height = height;
            
        pd.transPos = zeros(3, numSteps);
        pd.transVel = zeros(3, numSteps);
        pd.transAcc = zeros(3, numSteps);

        pd.rotPos = zeros(3, numSteps);
        pd.rotVel = zeros(3, numSteps);
        pd.rotAcc = zeros(3, numSteps);

        pd.q=zeros(4, numSteps);
        end
        
        function localV = toLocal(globalV)
            localV = rotMatrix\globalV;
        end
        function globalV = toGlobal(localV)
            globalV = rotMatrix*localV;
        end
        function [] = updateTranslational(self,accel)
            self.transAcc(:,n) = accel;
            self.transVel(:,n) = self.transVel(:,n-1) + timestep*self.transAcc(:,n);
            self.transPos(:,n) = self.transPos(:,n-1) + timestep*self.transVel(:,n);            
        end
    end
end

    