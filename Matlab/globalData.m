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
        TEMPERATURE = 350 % K
        TUBE_PRESSURE = 1000 % Pa
        GAS_CONSTANT = 8.314 % Pa * m^3 / K
        airDensity
    end
    methods
        function globals = globalData()
            globals.numSteps = globals.runtime / globals.timestep;
            globals.airDensity = globals.TUBE_PRESSURE...
                /(globals.GAS_CONSTANT*globals.TEMPERATURE);
        end        
    end
end
