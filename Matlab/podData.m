classdef podData
    properties
        mass
        tensor
        length
        width
        height
        collisionPoints
        leftSkate
        rightSkate
        leftRailWheels
        rightRailWheels
    end
    methods
        function pd = podData()
            pd.mass = 750;
            pd.length = 6;
            pd.width = 1;
            pd.height = 1;
            pd.tensor=[   1.0/12*pd.mass*(pd.height^2 + pd.width^2) 0 0; ...
                            0 1.0/12*pd.mass*(pd.height^2 + pd.length^2) 0; ...
                            0 0 1.0/12*pd.mass*(pd.length^2 + pd.width^2)];
                        
            pd.collisionPoints =[pd.length/2   pd.width/2  -pd.height/2; ...
                         pd.length/2  -pd.width/2  -pd.height/2; ...
                        -pd.length/2   pd.width/2  -pd.height/2; ...
                        -pd.length/2  -pd.width/2  -pd.height/2; ...
                         pd.length/2   pd.width/2   pd.height/2; ...
                         pd.length/2  -pd.width/2   pd.height/2; ...
                        -pd.length/2   pd.width/2   pd.height/2; ...
                        -pd.length/2  -pd.width/2   pd.height/2]';
        end
        
    end
end

    