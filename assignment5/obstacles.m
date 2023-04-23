% clear all
% 
% [c,s] = get_obstacles();
% figHandle = figure;
%     ax = gca;
%     hold all;
% for i = 3:length(c)
%     show(c(i), "Parent", ax);
% end
% 
% for i = 1:length(s)
%     show(s(i), "Parent", ax);
% end
% axis equal;
% xlabel('x')
% ylabel('y')
% zlabel('z')

function [casing,bottom_shaft] = obstacles() 
    casing = add_casing();
    bottom_shaft = add_bottom_shaft();
end

function casing = add_casing()
    %front casing dimensions
    % x,y,z
    th = 0.05; %thickness of all the panels
    %dimensions for the front and back panel
    fcd = [0.66,0.65,th];

    %dimensions for side panel approximation
    s_b_dim = [th,0.4,0.42];
    s_t_dim = [th,0.17,0.42];
    s_lr_dim = [th,0.16,0.13];

    %positions for the front and back panels
    %fcd_pose = [0.25,-(0.48-fcd(2)/2),0.42];
    fcd_pose = [fcd(1)/2,fcd(2)/2,fcd(3)/2];
    %bcd_pose = [0.25,-(0.48-fcd(2)/2),-0.42];
    bcd_pose = [fcd(1)/2,fcd(2)/2,fcd(3)+s_b_dim(2)];

    %positions for the many side panels
    side1_x = fcd(1)-th/2;
    s1_b_pose = [side1_x, s_b_dim(2)/2, th+s_b_dim(3)/2];
    s1_t_pose = [side1_x, s_b_dim(2)+s_lr_dim(2)+s_t_dim(2)/2, th+s_b_dim(3)/2];
    s1_r_pose = [side1_x, s_b_dim(2)+s_lr_dim(2)/2, th+s_t_dim(3)-s_lr_dim(3)/2];
    s1_l_pose = [side1_x, s_b_dim(2)+s_lr_dim(2)/2, th+s_lr_dim(3)/2];

    side2_x = th/2;
    s2_b_pose = [side2_x, s_b_dim(2)/2, th+s_b_dim(3)/2];
    s2_t_pose = [side2_x, s_b_dim(2)+s_lr_dim(2)+s_t_dim(2)/2, th+s_b_dim(3)/2];
    s2_r_pose = [side2_x, s_b_dim(2)+s_lr_dim(2)/2, th+s_t_dim(3)-s_lr_dim(3)/2];
    s2_l_pose = [side2_x, s_b_dim(2)+s_lr_dim(2)/2, th+s_lr_dim(3)/2];

    %creating obstacles for all of the casing
    front_casing = collisionBox(fcd(1),fcd(2),fcd(3));
    front_casing.Pose = trvec2tform(fcd_pose);
    back_casing = collisionBox(fcd(1),fcd(2),fcd(3));
    back_casing.Pose = trvec2tform(bcd_pose);

    
    side1_bottom = collisionBox(s_b_dim(1),s_b_dim(2),s_b_dim(3));
    side1_bottom.Pose = trvec2tform(s1_b_pose);

    side2_bottom = collisionBox(s_b_dim(1),s_b_dim(2),s_b_dim(3));
    side2_bottom.Pose = trvec2tform(s2_b_pose);

    side1_top = collisionBox(s_t_dim(1),s_t_dim(2),s_t_dim(3));
    side1_top.Pose = trvec2tform(s1_t_pose);

    side2_top = collisionBox(s_t_dim(1),s_t_dim(2),s_t_dim(3));
    side2_top.Pose = trvec2tform(s2_t_pose);

    side1_left = collisionBox(s_lr_dim(1),s_lr_dim(2),s_lr_dim(3));
    side1_left.Pose = trvec2tform(s1_l_pose);

    side1_right = collisionBox(s_lr_dim(1),s_lr_dim(2),s_lr_dim(3));
    side1_right.Pose = trvec2tform(s1_r_pose);

    side2_left = collisionBox(s_lr_dim(1),s_lr_dim(2),s_lr_dim(3));
    side2_left.Pose = trvec2tform(s2_l_pose);

    side2_right = collisionBox(s_lr_dim(1),s_lr_dim(2),s_lr_dim(3));
    side2_right.Pose = trvec2tform(s2_r_pose);

    side1_casing = [side1_top,side1_bottom,side1_left,side1_right];
    side2_casing = [side2_top,side2_bottom,side2_left,side2_right];

    casing = {front_casing,back_casing,side1_top,side1_bottom,side1_left,side1_right,side2_top,side2_bottom,side2_left,side2_right};

end

function gear_shaft = add_bottom_shaft()
    gear_width = 0.05;
    shaft_length = 0.66;
    shaft_radius = 0.072/2;
    y_pos = 0.25;
    z_pos = 0.21;
    shaft = collisionCylinder(shaft_radius, shaft_length);
    gear1 = collisionCylinder(0.28/2, gear_width);
    gears2 = collisionCylinder(0.247/2, gear_width*2);
    gears3 = collisionCylinder(0.16/2, gear_width*2+0.076);
    
    angles = pi/2;
    axis = "roty";
    %positions
    % shaft.Pose = trvec2tform([shaft_length/2, y_pos, z_pos]);
    % gear1.Pose = trvec2tform([0.06, y_pos, z_pos]);
    % gears2.Pose = trvec2tform([shaft_length-0.318, y_pos, z_pos]);
    % gears3.Pose = trvec2tform([shaft_length-0.402/2, y_pos, z_pos]);

    shaft.Pose = se3(angles, axis,"XYZ",[shaft_length/2, y_pos, z_pos]);
    gear1.Pose = se3(angles,axis,"XYZ",[0.06, y_pos, z_pos]);
    gears2.Pose = se3(angles,axis,"XYZ",[shaft_length-0.318, y_pos, z_pos]);
    gears3.Pose = se3(angles,axis,"XYZ",[shaft_length-0.402/2, y_pos, z_pos]);

    gear_shaft = {shaft, gear1, gears2, gears3};

end
%todo better side representation
