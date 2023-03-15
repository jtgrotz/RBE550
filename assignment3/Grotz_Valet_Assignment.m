clear all 
clc
%create occupancy map
map = imread("parkingmap.png");
bin_map = map(:,:,1) < 1;
bin_occ_map = binaryOccupancyMap(bin_map, 42); %approximate cells per meter res
vehicle_map = vehicleCostmap(bin_occ_map);

%%vehicle models
%differential Drive
Axle_width = 1.2;
WheelRadius = 0.6;
WheelBase = 2.8;
Rear_over = 0.4;
Front_over = 0.4;
distance = 0.75;
inputs = [distance 0; distance pi/4; distance pi/2;
    0 pi/12; 0 pi/4; 0 pi/2;
    distance -pi/4; distance -pi/2;];
    %-1 0; -1 1.5; -1 3;
    % -1 -1.5; -1 -3];

% %Ackerman Steering
% WheelBase = 2.8;
% Axle_width = 1.2;
% Rear_over = 0.4;
% Front_over = 0.4;
%inputs = [1 pi/3; 1 pi/6; 1 pi/12; 1 0;
 %           1 -pi/3; 1 -pi/6; 1 -pi/12; 1 0;
 %           -1 pi/3; -1 pi/6; -1 pi/12; -1 0;
 %           -1 -pi/3; -1 -pi/6; -1 -pi/12; -1 0;];
% 
% %Trailer truck
% WheelBase = 3;
% Axle_width = 1.75;
% Rear_over = 0.4;
% Front_over = 0.4;

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

%% matlab path planning
validator = validatorOccupancyMap;
validator.Map = bin_occ_map;
hybridPlanner = plannerHybridAStar(validator,MinTurningRadius=2,MotionPrimitiveLength=0.5);
startPose = [3 8.5 0]; % [meters, meters, radians]
goalPose = [4.5 1 pi];
refpath = plan(hybridPlanner,startPose,goalPose);
show(hybridPlanner)

%% my path planning
astar = hybridAstar();
astar.inputs = inputs;
astar.timescale = [0 1];
kin = ackerman_steering_kinematics(WheelRadius,Axle_width,WheelBase,Rear_over,Front_over);
[path,c_out,searched] = astar.search(vehicle_map,kin,[3 8.5 0],[4.5 1 pi]);
hold on
plot_path(path);



%% extra functions

function plot_path(path)
    plot(vehicle_map);
    hold on
    plot(path(:,1),path(:,2));
    quiver(path(:,1),path(:,2),cos(path(:,3)),sin(path(:,3)),0.3);
end

function plot_integrated_path()
end

function plot_robot()
end

function plot_start_end()
end