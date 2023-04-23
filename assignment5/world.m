classdef world
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        obstacles = {}
    end

    methods
        function obs = get.obstacles(obj)
            obs = obj.obstacles;
        end

        function obj = add_obstacles(obj,obstacle_array)
            obj.obstacles{end+1} = (obstacle_array);
        end
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end