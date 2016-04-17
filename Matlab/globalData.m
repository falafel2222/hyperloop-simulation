classdef globalData

    properties
        
        %%% TIMING %%%
        timestep = .001 %s
        kalmanTimestep = .001 %s
        runtime = 15 %s
        numSteps

        %%% SIMULATION OPTIONS %%%
        randomNoise=false;
        noiseModifier=0.00000001;
            
        %%% TUBE SPECS %%%
        pusherForce = 17640 % newtons
        pusherDistance = 273 % m
        
        %%% NATURAL CONSTANTS %%%
        gravity = 9.81 % m/s^2
    end
    methods
        function globals = globalData()
            globals.numSteps = globals.runtime / globals.timestep;
        end        
    end
end
