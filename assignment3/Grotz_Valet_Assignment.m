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

% %Ackerman Steering
% WheelBase = 2.8;
% Axle_width = 1.2;
% Rear_over = 0.4;
% Front_over = 0.4;
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

%% actual path planning
validator = validatorOccupancyMap;
validator.Map = bin_occ_map;
hybridPlanner = plannerHybridAStar(validator,MinTurningRadius=2,MotionPrimitiveLength=0.5);
startPose = [3 8.5 0]; % [meters, meters, radians]
goalPose = [4.5 1 pi];
refpath = plan(hybridPlanner,startPose,goalPose);
show(hybridPlanner)
