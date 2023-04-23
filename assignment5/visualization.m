clear all

t = transmission();
[c,s] = obstacles();
figHandle = figure;
    ax = gca;
    hold all;
for i = 3:length(c)
    show(c{i}, "Parent", ax);
end

for i = 1:length(s)
    show(s{i}, "Parent", ax);
end
axis equal;
xlabel('x')
ylabel('y')
zlabel('z')

% conf = homeConfiguration(t);
% conf(1).JointPosition = 0.42/2+0.05;
% conf(2).JointPosition = 0.460;
% conf(3).JointPosition = 0.660-0.076-0.384-0.04;
% conf(4).JointPosition = 0;
% conf(5).JointPosition = -pi/2; %joint offset because of DH weirdness
% conf(6).JointPosition = 0;

conf = [0.42/2+0.05,0.460,0.660-0.076-0.384-0.04,0,-pi/2,0];

show(t,conf,"Parent",ax);

%create robot class, and add transmission robot model
my_robot = robot();
my_robot.robot_model = t;
my_robot.initial_config = conf;

% create world, and add created obstacles of bottom shaft and case
my_world = world();
my_world = add_obstacles(my_world, c);
my_world = add_obstacles(my_world, s);

%states are in the form x y z thetax, thetay, thetaz;
minstates = [0,0,0,-pi,-pi,-pi];
maxstates = [1.5,1.5,1.5,pi,pi,pi];

rrt(my_robot,my_world,[1,1,1,0,0,0],minstates,maxstates,300,0.1);
