classdef robot
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        robot_model
        initial_config
    end

    methods
        function is_collision = check_collision(obj,world,config)
            obs = world.get_obstacles(obj.robot_model);
            %re-arrange config
            new_config = config;
            is_colliding = checkCollision(obj.robot_model,new_config, obs,...
                'IgnoreSelfCollision', 'on');
            is_collision = any(is_colliding);
        end

        function obj = set.robot_model(obj,model)
            obj.robot_model = model;
        end

        function m = get.robot_model(obj)
            m = obj.robot_model;
        end
    end
end