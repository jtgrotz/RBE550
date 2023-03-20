classdef simulator
    properties
        vehicle_map
        WheelBase
        Axle_width 
        WheelRadius
        Rear_over
        Front_over 
        hitch_length 
        distance
        inputs
        car_shape
        trailer_shape


    end

    methods
        function [path,c_out,searched] = run_pathplanning(obj,start,goal,has_trailer)
            %set vehicle parameters for collision
            l = obj.WheelBase * 1.2;
            w = obj.Axle_width*1.2;
            h = 1.4;
            r_o = (l-obj.WheelBase)/2;
            f_o = (l-obj.WheelBase)/2;
            veh_dims = vehicleDimensions(l, w, h,"FrontOverhang", f_o, "RearOverhang", r_o);
            coll_checker = inflationCollisionChecker(veh_dims,3);
            coll_checker.InflationRadius = coll_checker.InflationRadius*0.8;
            vehicle_map.CollisionChecker = coll_checker;
            
            %car for plotting
            X_d = [-obj.Rear_over,obj.WheelBase+obj.Front_over,obj.WheelBase+obj.Front_over,-obj.Rear_over];
            Y_d = [-obj.Axle_width/2,-obj.Axle_width/2,obj.Axle_width/2,obj.Axle_width/2];
            obj.car_shape = polyshape(X_d',Y_d');
            obj.trailer_shape = polyshape((X_d/2)',Y_d');

            if (has_trailer == 0)
                astar = hybridAstar();
                astar.inputs = inputs;
                astar.timescale = [0 1];
                kin = ackerman_steering_kinematics(WheelRadius,Axle_width,WheelBase,Rear_over,Front_over);
                %kin = differential_kinematics(WheelRadius,Axle_width,WheelBase,Rear_over,Front_over);
                [path,c_out,searched] = astar.search(vehicle_map,kin,[1 8.5 0],[4.5 1 0]);
            else
                astar = hybridAstar();
                astar.inputs = inputs;
                astar.timescale = [0 1];
                kin = trailer_kinematics(WheelRadius,Axle_width,WheelBase,Rear_over,Front_over,hitch_length);
                [path,c_out,searched] = astar.search(vehicle_map,kin,[6 8.7 0 0],[7.5 1 pi pi]);
            end
        end

        function visualization(path, c_out, map, has_trailer)
            if (has_trailer == 0)
                plot_path(map, path);
                integrated_path = plot_integrated_path(map,kin,path,c_out,astar.timescale);
                plot_robot(car_shape,integrated_path,map)
            else
            end
        end  
            
        %% extra functions
        function plot_path(map, path)
            figure;
            plot(map);
            hold on
            plot(path(:,1),path(:,2));
            quiver(path(:,1),path(:,2),cos(path(:,3)),sin(path(:,3)),0.3);
            hold off
        end
        
        
        function plot_path_trailer(map, path, hitch_length)
            figure;
            plot(map);
            hold on
            plot(path(:,1),path(:,2));
            quiver(path(:,1),path(:,2),cos(path(:,3)),sin(path(:,3)),0.3);
            t_x = path(:,1)-(hitch_length*cos(-path(:,4)));%x2 =  x1-d1*cos(theta1)
            t_y = path(:,2)-(hitch_length*sin(-path(:,4)));%y2 =  y1 -d1*sin(theta1)
            plot(t_x,t_y,'r');
            quiver(t_x,t_y,cos(-path(:,4)),sin(-path(:,4)),0.3);
        
            hold off
        end
        function total_path = plot_integrated_path(map,robot_kinematics,path,c_out,ts)
        
        figure;
        plot(map);
        hold on
        total_path = [];
            for i = 1:length(path)
                robot_kinematics.current_position = path(i,:)';
                [~,~,pts] = robot_kinematics.integrate_position(c_out(i,1),c_out(i,2),ts);
                plot(pts(:,1),pts(:,2));
                total_path = [total_path; pts];
            end
            hold off
        end
        
        function plot_robot(shape1, path, map)
            figure;
            for i = 1:length(path)
                plot(map)
                hold on
                moved_shape = move_car(shape1,path(i,:));
                plot(moved_shape);
                hold off
                pause(0.05);
            end
        end
        
        function plot_robot_trailer(shape1, shape2, path, map, hitch_length)
            figure;
            for i = 1:length(path)
                plot(map)
                hold on
                moved_shape = move_car(shape1,path(i,1:3));
                plot(moved_shape);
                t_x = path(i,1)-(hitch_length*cos(path(i,4)));%x2 =  x1-d1*cos(theta1)
                t_y = path(i,2)-(hitch_length*sin(-path(i,4)));%y2 =  y1 -d1*sin(theta1)
                moved_shape2 = move_car(shape2,[t_x,t_y,(-path(i,4))]);
                plot(moved_shape2);
                plot([path(i,1),t_x],[path(i,2),t_y]);
                hold off
                pause(0.05);
            end
        end
        
        function plot_searched(searched)
            hold on
            for i = length(searched)
                p = str2num(searched(i))
                plot(p(1),p(2));
            end
        end
        
        function plot_start_end()
        end
        
        function m = move_car(car,pose)
            m = rotate(translate(car,[pose(1),pose(2)]),(pose(3)*180/pi),[pose(1),pose(2)]);
        end
            end
end