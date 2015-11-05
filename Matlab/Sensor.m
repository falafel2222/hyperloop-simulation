classdef Sensor
    properties
        refreshRate % hertz
        error % percentage error
        location % where the sensor is with respect to the center of the pod
        direction % what direction the sensor points (for lasers)
    end
    methods
        % base constructor
        function s = Sensor(refreshRate,error,location,direction)
            s.refreshRate = refreshRate;
            s.error = error;
            s.location = location;
            s.direction = direction;
        end
    end
end
