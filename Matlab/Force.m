classdef Force
    %Force Information about a force which acts upon the pod
    %   Detailed explanation goes here
    
    properties
        isLocal         %true or false
        location        %always in local frame
        components      %depends on whether isLocal
    end
 
    methods
        function force = Force(local,location,components)
           force.isLocal=local;
           force.location=location;
           force.components=components;
        end
        
        function [] = toGlobal(self,rotMatrix)
            if self.isLocal
               self.isLocal=false;
               self.components=self.rotMatrix*self.components;
            end
        end
        
        function [] = toLocal(self,rotMatrix)
           if ~self.isLocal
               self.isLocal=true;
               self.components = rotMatrix\self.components;
           end
        end
    end
    
end

