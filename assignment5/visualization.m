clear all

t = transmission();
[c,s] = obstacles();
figHandle = figure;
    ax = gca;
    hold all;
for i = 3:length(c)
    show(c(i), "Parent", ax);
end

for i = 1:length(s)
    show(s(i), "Parent", ax);
end
axis equal;
xlabel('x')
ylabel('y')
zlabel('z')

conf = homeConfiguration(t);
conf(1).JointPosition = 0.42/2+0.05;
conf(2).JointPosition = 0.460;
conf(3).JointPosition = 0.660-0.076-0.384-0.04;
conf(4).JointPosition = 0;
%conf(5).JointPosition = 0;
%conf(6).JointPosition = 0;

show(t,conf,"Parent",ax);