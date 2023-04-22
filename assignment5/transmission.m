function transmission = transmission()
    %dh for 6dof on distance between points
     % dh = [0 pi/2 0 0;
     %     0 pi/2 0 pi/2;
     %    0 0 0 0;
     %     0 -pi/2 0 0;
     %    0 -pi/2 0 -pi/2;
     %    0 0 0 0];
        %a alpha d theta
       dh = [0 -pi/2 0 0;
         0 -pi/2 0 -pi/2;
        0 0 0 pi;
         0 -pi/2 0 0;
        0 pi/2 0 +pi/2;
        0 0 0 0];

    %dimensions for different primatives of the transmission
    main_length = 0.660;
    a1 = 0.150;
    p1 = 0.04+a1/2;
    a2 = 0.300;
    p2 = p1+0.3/2;

    %create body
    transmission = rigidBodyTree;
    %tf matrix for the different collisions at the end of the movement
    tf = [1,0,0,0
        0,1,0,0;
        0,0,1,main_length/2;
        0,0,0,1];

    %create 6dof of movement
    % body1 = rigidBody('body1');
    % jnt1 = rigidBodyJoint('z_rot','revolute');
    % body2 = rigidBody('body2');
    % jnt2 = rigidBodyJoint('y_rot','revolute');
    % body3 = rigidBody('body3');
    % jnt3 = rigidBodyJoint('x_rot','revolute');
    % body4 = rigidBody('body4');
    % jnt4 = rigidBodyJoint('x_trans','prismatic');
    % body5 = rigidBody('body5');
    % jnt5 = rigidBodyJoint('y_trans','prismatic');
    % body6 = rigidBody('body6');
    % jnt6 = rigidBodyJoint('z_trans','prismatic');

     body1 = rigidBody('body1');
    jnt1 = rigidBodyJoint('z_trans','prismatic');
    body2 = rigidBody('body2');
    jnt2 = rigidBodyJoint('y_trans','prismatic');
    body3 = rigidBody('body3');
    jnt3 = rigidBodyJoint('x_trans','prismatic');
    body4 = rigidBody('body4');
    jnt4 = rigidBodyJoint('x_rot','revolute');
    body5 = rigidBody('body5');
    jnt5 = rigidBodyJoint('y_rot','revolute');
    body6 = rigidBody('body6');
    jnt6 = rigidBodyJoint('z_rot','revolute');

    setFixedTransform(jnt1,dh(1,:),'dh');
    setFixedTransform(jnt2,dh(2,:),'dh');
    setFixedTransform(jnt3,dh(3,:),'dh');
    setFixedTransform(jnt4,dh(4,:),'dh');
    setFixedTransform(jnt5,dh(5,:),'dh');
    setFixedTransform(jnt6,dh(6,:),'dh');
    body1.Joint = jnt1;
    body2.Joint = jnt2;
    body3.Joint = jnt3;
    body4.Joint = jnt4;
    body5.Joint = jnt5;
    body6.Joint = jnt6;

    addBody(transmission,body1,'base')
    addBody(transmission,body2,'body1')
    addBody(transmission,body3,'body2')
    addBody(transmission,body4,'body3')
    %addBody(transmission,body5,'body4')
    %addBody(transmission,body6,'body5')

       %add collision for the actual transmission.
    mainshaft = rigidBody('body7');
    mainshaft_jnt = rigidBodyJoint('jnt7','fixed');
    setFixedTransform(mainshaft_jnt,[0,0,0,0],"dh");
    mainshaft.Joint = mainshaft_jnt;
    addCollision(mainshaft,"cylinder",[0.072/2,0.660],tf);
   % addVisual(mainshaft,"cylinder",[0.072/2,0.660],tf);

    tf(3,4) = p1;
    sub_one = rigidBody('body8');
    sub_one_jnt = rigidBodyJoint('jnt8','fixed');
    setFixedTransform(mainshaft_jnt,[a1,0,a1,0],"dh");
    sub_one.Joint = sub_one_jnt;
    addCollision(sub_one,"cylinder",[0.212/2,0.150],tf);
    %addVisual(sub_one,"cylinder",[0.212/2,0.150],tf);

    tf(3,4) = p2;
    sub_two = rigidBody('body9');
    sub_two_jnt = rigidBodyJoint('jnt9','fixed');
    setFixedTransform(mainshaft_jnt,[a2,0,a2,0],"dh");
    sub_two.Joint = sub_two_jnt;
    addCollision(sub_two,"cylinder",[0.260/2,0.234],tf);
   % addVisual(sub_two,"cylinder",[0.260/2,0.234],tf);

   % addBody(transmission,mainshaft,'body6');
   % addBody(transmission,sub_one,'body7');
    %addBody(transmission,sub_two,'body8');

    %transmission.Bodies

end

