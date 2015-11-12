classdef Sensor
    properties
        refreshRate % hertz
        error % percentage error        
    end
    methods
        % base constructor
        function s = Sensor(refreshRate,error)
            if ~nargin == 0
                s.refreshRate = refreshRate;
                s.error = error;
            end
        end
    end
end
