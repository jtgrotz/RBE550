classdef ackerman_steering_kinematics
     properties
        wheel_radius
        axel_width
        wheelbase
        rear_over
        front_over
        current_position = [0;0;0];
    end

    methods
        function obj = ackerman_steering_kinematics(wheel_radius, axel_width, wheelbase, rear_over, front_over)
        obj.axel_width = axel_width;
        obj.wheel_radius = wheel_radius;
        obj.front_over = front_over;
        obj.rear_over = rear_over;
        obj.wheelbase = wheelbase;
        end

        function [final_point,t,q] = integrate_position(obj,lin_velocity,steering_angle,time_step)
            [t,q] = ode45(@(t,q) diffkin(obj,q,lin_velocity,steering_angle,obj.wheelbase), time_step, obj.current_position);
            final_point = q(end,:);
        end

        %basic collision checker for defined cost map. 
        %costmap is vehicle cost map object
        %points is a nx3 matrix of points
        function coll = check_collision(obj,costmap,points)
            %% 
            limits = costmap.MapExtent;
            x_oob = any(points(:,1) >= (limits(2)*0.98)) || any(points(:,1) <= (limits(1)*0.98));
            y_oob = any(points(:,2) >= (limits(4)*0.98)) || any(points(:,2) <= (limits(3)*0.98));
            if (x_oob || y_oob)
                coll = 1;
            else
                try
                    coll = any(checkOccupied(costmap,points));
                catch
                    coll = 0;
                end
            end
        end
    end

    methods (Access = private)
        function dqdt = diffkin(obj,q,v,steering_angle,L)
            dqdt = zeros(3,1);
            dqdt(1) = v*cos(q(3));
            dqdt(2) = v*sin(q(3));
            dqdt(3) = (v/L)*tan(steering_angle);
        end
    end

end