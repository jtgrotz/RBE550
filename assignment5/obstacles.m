front_casing = collisionBox(fcx,fcy,fcz);
back_casing = collisionBox(bcx,bcy,bcz);
%todo better side representation
side_casing = collisionBox(scx,scy,scz);

bottom_gears = collisionCylinder(bgr, bgl);

t = pi/12:pi/12:2*pi;
pgon = polyshape({[-0.5 -0.5 0.5 0.5], 0.25*cos(t)}, ...
                 {[0.5 -0.5 -0.5 0.5], 0.25*sin(t)})

vertices = [pgon.Vertices, zeros(length(pgon.Vertices),1);pgon.Vertices, ones(length(pgon.Vertices),1)]

hole = multicylinder([9 10],10,"Void",[true,false])