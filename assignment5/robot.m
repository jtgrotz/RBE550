classdef robot
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        robot_model
        initial_config
        current_config
    end

    methods
        function is_collision = check_collision(obj,world,config)
            obs = world.obstacles();
            obs_list_length = length(obs);
            for i = 1:obs_list_length
                obs_set = obs{i};
                %re-arrange config
                new_config = config;
                mod_config = set_config_row(obj,new_config);
                is_colliding = checkCollision(obj.robot_model,mod_config, obs_set,"IgnoreSelfCollision", "on");
                is_collision = any(is_colliding);
                if (is_collision == true)
                    break;
                end
            end
        end
        
        %config vector set as x,y,z,thetax,thetay,thetaz
        function conf = set_config_struct(obj,vector)
            conf = obj.initial_config;
            %robot states are z,y,x,thetax,thetay,thetaz
            conf(1).JointPosition = vector(3);
            conf(2).JointPosition = vector(2);
            conf(3).JointPosition = vector(1);
            conf(4).JointPosition = vector(4);
            conf(5).JointPosition = vector(5)-pi/2; %joint offset for dh
            conf(6).JointPosition = vector(6);
        end

        function conf = set_config_row(obj,vector)
            conf = [vector(3),vector(2),vector(1),vector(4),vector(5),vector(6)];
        end

        function obj = set.robot_model(obj,model)
            obj.robot_model = model;
        end

        function m = get.robot_model(obj)
            m = obj.robot_model;
        end

        function c = get_initial_config_struct(obj)
            [z,y,x,tx,ty,tz] = obj.initial_config.JointPosition;
            c = [x,y,z,tx,ty,tz];
        end 

        function c = get_initial_config_row(obj)
            arr = obj.initial_config;
            c = [arr(3),arr(2),arr(1),arr(4),arr(5),arr(6)];
        end
    end
end