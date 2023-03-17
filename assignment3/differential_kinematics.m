classdef differential_kinematics
    properties
        wheel_radius
        axel_width
        wheelbase
        rear_over
        front_over
        current_position
    end

    methods
        function obj = differential_kinematics(wheel_radius, axel_width, wheelbase, rear_over, front_over)
        obj.axel_width = axel_width;
        obj.wheel_radius = wheel_radius;
        obj.front_over = front_over;
        obj.rear_over = rear_over;
        obj.wheelbase = wheelbase;
        end

        function [final_point,t,q] = integrate_position(obj,lin_velocity,ang_velocity,time_step)
            [t,q] = ode45(@(t,q) diffkin(obj,q,lin_velocity,ang_velocity), time_step, obj.current_position);
            final_point = q(end,:);
        end
        %basic collision checker for defined cost map. 
        %costmap is vehicle cost map object
        %points is a nx3 matrix of points
        function coll = check_collision(obj,costmap,points)
            limits = costmap.MapExtent;
            x_oob = any(points(:,1) >= (limits(2)*0.98)) || any(points(:,1) <= (limits(1)*0.98));
            y_oob = any(points(:,2) >= (limits(4)*0.98)) || any(points(:,2) <= (limits(3)*0.98));
            if (x_oob || y_oob)
                coll = 1;
            else
                coll = any(checkOccupied(costmap,points));
            end
            
        end
    end

    methods (Access = private)
        function dqdt = diffkin(obj,q,v,w)
            dqdt = zeros(3,1);
            dqdt(1) = v*cos(q(3));
            dqdt(2) = v*sin(q(3));
            dqdt(3) = w;
        end
    end

end