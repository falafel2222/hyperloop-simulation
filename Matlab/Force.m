classdef Force
    %Force Information about a force which acts upon the pod
    %   Detailed explanation goes here
    
    properties
        isLocal         %true or false
        location        %always in local frame
        components      %depends on whether isLocal
    end
 
    methods
        function force = Force(isLocal,location,components,noiseModifier)
           if ~nargin == 0
               force.isLocal=isLocal;
               force.location=location;
               force.components=components*(1+noiseModifier*(1-2*rand()));
           end
        end
        
        function self = toGlobal(self,rotMatrix)
            if self.isLocal
               self.isLocal=false;
               self.components=rotMatrix*self.components;
            end
        end
        
        function self = toLocal(self,rotMatrix)
           if ~self.isLocal
               self.isLocal=true;
               self.components = rotMatrix\self.components;
           end
        end
    end
    
end

