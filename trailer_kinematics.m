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
        function obj = trailer_kinematics(wheel_radius, axel_width, wheelbase, rear_over, front_over, hitch_length)
        obj.axel_width = axel_width;
        obj.wheel_radius = wheel_radius;
        obj.front_over = front_over;
        obj.rear_over = rear_over;
        obj.wheelbase = wheelbase;
        obj.hitch_length = hitch_length;
        end

        function [final_point,t,q] = integrate_position(obj,lin_velocity,steering_angle,time_step)
            [t,q] = ode45(@(t,q) diffkin(obj,q,lin_velocity,steering_angle,obj.wheelbase,obj.hitch_length), time_step, obj.current_position);
            final_point = q(end,:);
        end

        function coll = check_collision(obj,map,point)
            %x2 =  x1-d1*cos(theta1)
            %y2 =  y1 -d1*sin(theta1)
        end
    end

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