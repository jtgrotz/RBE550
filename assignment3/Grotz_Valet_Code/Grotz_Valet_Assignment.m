clear all 
clc
%create occupancy map
%map = imread("parkingmap.png");
map = imread("parkingmap2.png");

bin_map = map(:,:,1) < 1;
bin_occ_map = binaryOccupancyMap(bin_map, 42); %approximate cells per meter res
vehicle_map = vehicleCostmap(bin_occ_map);

%% vehicle models
%differential Drive
Axle_width = 1.2;
WheelRadius = 0.6;
WheelBase = 2.8;
Rear_over = 0.2;
Front_over = 0.2;
distance = 0.75;
inputs = [distance 0; distance pi/6; distance pi/3; distance pi/2;
    0 pi/18; 0 pi/6; 0 pi/3; 0 pi/2;
    -distance 0; distance -pi/6 ;distance -pi/3; distance -pi/2;
    -distance -pi/12; -distance -pi/6; -distance -pi/3; -distance -pi/2;
    -distance pi/12; -distance pi/6; -distance pi/3; -distance pi/2;];

%%
% %Ackerman Steering
WheelBase = 2.8;
Axle_width = 1.2;
WheelRadius = 0.6;
Rear_over = 0.2;
Front_over = 0.2;
distance = 0.75;
inputs = [distance pi/3; distance pi/6; distance pi/2.3; distance 0;
           distance -pi/3; distance -pi/6; distance -pi/2.3; distance 0;
           -distance pi/3; -distance pi/6; -distance pi/2.2; -distance 0;
           -distance -pi/3; -distance -pi/6; -distance -pi/2.3; -distance 0;];

%%

%Trailer truck
WheelBase = 3;
Axle_width = 1.75;
WheelRadius = 0.6;
Rear_over = 0.2;
Front_over = 0.2;
hitch_length = 3;
distance = 1.5;
inputs = [distance pi/3; distance pi/6; distance 0;
    distance -pi/3; distance -pi/6; distance 0;
    -distance pi/3; -distance pi/6; -distance 0;
    -distance -pi/3; -distance -pi/6; -distance 0;];

%%
%Creation of vehicle for collision
%set vehicle parameters for collision
l = WheelBase * 1.2;
w = Axle_width*1.2;
h = 1.4;
r_o = (l-WheelBase)/2;
f_o = (l-WheelBase)/2;
veh_dims = vehicleDimensions(l, w, h,"FrontOverhang", f_o, "RearOverhang", r_o);
coll_checker = inflationCollisionChecker(veh_dims,3);
coll_checker.InflationRadius = coll_checker.InflationRadius*0.8;
vehicle_map.CollisionChecker = coll_checker;

%car for plotting
X_d = [-Rear_over,WheelBase+Front_over,WheelBase+Front_over,-Rear_over];
Y_d = [-Axle_width/2,-Axle_width/2,Axle_width/2,Axle_width/2];
car_shape = polyshape(X_d',Y_d');
trailer_shape = polyshape((X_d/2)',Y_d');

%% matlab path planning
%used as a baseline performance metric.
% validator = validatorOccupancyMap;
% validator.Map = bin_occ_map;
% hybridPlanner = plannerHybridAStar(validator,MinTurningRadius=2,MotionPrimitiveLength=0.5);
% startPose = [3 8.5 0]; % [meters, meters, radians]
% goalPose = [4.5 1 pi];
% refpath = plan(hybridPlanner,startPose,goalPose);
% show(hybridPlanner)

%% custom differential path planning
astar = hybridAstar();
astar.inputs = inputs;
astar.timescale = [0 1];
kin = differential_kinematics(WheelRadius,Axle_width,WheelBase,Rear_over,Front_over);
[path,c_out,searched] = astar.search(vehicle_map,kin,[1 8.5 0],[4.5 1 0]);
%after search plotting
plot_path(vehicle_map, path);
integrated_path = plot_integrated_path(vehicle_map,kin,path,c_out,astar.timescale);
plot_robot(car_shape,integrated_path,vehicle_map)

%% custom ackerman path planning
astar = hybridAstar();
astar.inputs = inputs;
astar.timescale = [0 1];
kin = ackerman_steering_kinematics(WheelRadius,Axle_width,WheelBase,Rear_over,Front_over);
[path,c_out,searched] = astar.search(vehicle_map,kin,[1 8.5 0],[4.5 1 0]);
%%
%after search plotting
plot_path(vehicle_map, path);
integrated_path = plot_integrated_path(vehicle_map,kin,path,c_out,astar.timescale);
plot_robot(car_shape,integrated_path,vehicle_map)

%% custom trailer path planning
astar = hybridAstar();
astar.inputs = inputs;
astar.timescale = [0 1];
astar.heading_tolerance = 30;
kin = trailer_kinematics(WheelRadius,Axle_width,WheelBase,Rear_over,Front_over,hitch_length);
[path,c_out,searched] = astar.search(vehicle_map,kin,[13 4 -pi/2 -pi/2],[7.5 1 pi pi]);
%after search plotting
%%
plot_path_trailer(vehicle_map,path,hitch_length);
integrated_path = plot_integrated_path(vehicle_map,kin,path,c_out,astar.timescale);
plot_robot_trailer(car_shape , trailer_shape, integrated_path, vehicle_map, hitch_length)

%% helper functions
%plots path of the robot and heading at each point.
function plot_path(map, path)
    figure;
    plot(map);
    hold on
    plot(path(:,1),path(:,2));
    quiver(path(:,1),path(:,2),cos(path(:,3)),sin(path(:,3)),0.3);
    hold off
end

%plots path but also accounts for trailer position
function plot_path_trailer(map, path, hitch_length)
    figure;
    plot(map);
    hold on
    plot(path(:,1),path(:,2));
    quiver(path(:,1),path(:,2),cos(path(:,3)),sin(path(:,3)),0.3);
    t_x = path(:,1)-(hitch_length*cos(path(:,4)));%x2 =  x1-d1*cos(theta1)
    t_y = path(:,2)-(hitch_length*sin(path(:,4)));%y2 =  y1 -d1*sin(theta1)
    plot(t_x,t_y,'r');
    quiver(t_x,t_y,cos(path(:,4)),sin(path(:,4)),0.3);

    hold off
end

%takes the saved control inputs and plots the integrated path.
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

%plots the robot shape primative for each point in the integrated path
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

%plots the robot and trailer shape primative for each point in the integrated path
function plot_robot_trailer(shape1, shape2, path, map, hitch_length)
    figure;
    for i = 1:length(path)
        plot(map)
        hold on
        moved_shape = move_car(shape1,path(i,1:3));
        plot(moved_shape);
        t_x = path(i,1)-(hitch_length*cos(path(i,4)));%x2 =  x1-d1*cos(theta1)
        t_y = path(i,2)-(hitch_length*sin(path(i,4)));%y2 =  y1 -d1*sin(theta1)
        moved_shape2 = move_car(shape2,[t_x,t_y,(path(i,4))]);
        plot(moved_shape2);
        plot([path(i,1),t_x],[path(i,2),t_y]);
        hold off
        pause(0.05);
    end
end

%plots every searched point
function plot_searched(searched)
    hold on
    for i = length(searched)
        p = str2num(searched(i))
        plot(p(1),p(2));
    end
end

%translates the robot primative by the given pose
function m = move_car(car,pose)
    m = rotate(translate(car,[pose(1),pose(2)]),(pose(3)*180/pi),[pose(1),pose(2)]);
end