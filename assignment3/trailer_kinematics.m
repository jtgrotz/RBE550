classdef trailer_kinematics
     properties
        wheel_radius
        axel_width
        wheelbase
        rear_over
        front_over
        hitch_length
        current_position = [0;0;0;0];
    end

    methods
        %init function
        function obj = trailer_kinematics(wheel_radius, axel_width, wheelbase, rear_over, front_over, hitch_length)
        obj.axel_width = axel_width;
        obj.wheel_radius = wheel_radius;
        obj.front_over = front_over;
        obj.rear_over = rear_over;
        obj.wheelbase = wheelbase;
        obj.hitch_length = hitch_length;
        end

            %integrates kinematics for control inputs over the given
            %time_step
        function [final_point,t,q] = integrate_position(obj,lin_velocity,steering_angle,time_step)
            [t,q] = ode45(@(t,q) diffkin(obj,q,lin_velocity,steering_angle,obj.wheelbase,obj.hitch_length), time_step, obj.current_position);
            final_point = q(end,:);
        end

            %checks for collision along the integration path.
            %trailer has four points, but the pose is only the first three
            %fourth point is used to derive trailer position.
            %return 1 for collision 
        function coll = check_collision(obj,costmap,points)
            %x2 =  x1-d1*cos(theta1)
            %y2 =  y1 -d1*sin(theta1)
            coll_car = any(checkOccupied(costmap,points(:,1:3)));
            trailer_points_x = points(:,1)-obj.hitch_length*cos(points(:,4));
            trailer_points_y = points(:,1)-obj.hitch_length*sin(points(:,4));
            trailer_points = [trailer_points_x,trailer_points_y,points(:,4)];
            coll_trailer = any(checkOccupied(costmap,trailer_points));
            coll = any([coll_car,coll_trailer]);

        end
    end

        %kinematic model for the trailer car
    methods (Access = private)
        function dqdt = diffkin(obj,q,v,steering_angle,L,d1)
            dqdt = zeros(4,1);
            dqdt(1) = v*cos(q(3));
            dqdt(2) = v*sin(q(3));
            dqdt(3) = (v/L)*tan(steering_angle);
            dqdt(4) = (v/d1)*sin(q(3)-q(4));
        end
    end

end