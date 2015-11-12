classdef Laser
    properties
        refreshRate % hertz
        error % percentage error
        location % where the sensor is with respect to the center of the pod
        direction % what direction the sensor points (for lasers)
    end
    methods
        % base constructor
        function l = Laser(refreshRate,error,location,direction)
            if ~nargin == 0
                l.refreshRate = refreshRate;
                l.error = error;
                l.location = location;
                l.direction = direction;
            end
        end
    end
end
