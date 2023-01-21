clc; clear;
shape1 = [0,0;0,1;0,2;0,3];
shape2 = [0,0;1,0;0,1;0,2];
shape3 = [0,0;0,1;0,2;1,1];
shape4 = [0,0;0,1;1,1;1,2];

shape_set = {shape1,shape2,shape3,shape4};

[gen_field, d] = generate_obstacles(128,0.5,shape_set);
imshow(gen_field);

function [field, actual_density] = generate_obstacles(field_size, density, shapes)
shape_size = 4;
%Create board (white spaces are 1 (empty), black spaces are 0 (are filled)
field = ones(field_size, field_size);
%determine how many shapes to place on the board
squares_to_fill = field_size^2*density;
number_of_shapes = round(squares_to_fill/shape_size);

%generate random coordinates for random shape type
%scale random number by fieldsize
rand_x_coord = ceil((field_size)*rand(number_of_shapes,1));
rand_y_coord = ceil((field_size)*rand(number_of_shapes,1));
rand_coords = [rand_x_coord,rand_y_coord];
rand_shapes = ceil((shape_size)*rand(number_of_shapes,1));

good_points = 0;
bad_points = 0;
%loop through every point
    for i = 1:number_of_shapes
        %see if piece can be placed on board
        point_coords = rand_coords(i,:);
        %finds shape and randomly inverts and/or rotates it
        chosen_shape = alter_shape(shapes{rand_shapes(i,1)});
        %point placed flag indicates if the block has been successfully
        %placed, otherwise keep trying new points.
        point_placed = 0;
        
        while(point_placed == 0)
            %see if piece can be placed on board
            bad_placement_flag = check_bad_placement(point_coords, chosen_shape,field);
            if bad_placement_flag == 0
              %fill the board
                field = write_to_board(point_coords,chosen_shape,field);
                point_placed = 1;
                good_points = good_points + 1;
            else
                %gen new point and try again
                point_coords = [ceil((field_size)*rand(1,1)),ceil((field_size)*rand(1,1))];
                bad_points = bad_points + 1;
            end
        end

    end
    actual_density = 1 - (sum(sum(field))/(length(field)*width(field)));
end

% Inverts the given piece if doesn't fit within the given space
function y = invert_piece(piece)
   y = -piece;
end

% Inverts the given piece if doesn't fit within the given space
function y = rotate_piece(piece)
    y = zeros(size(piece));
    y(:,2) = piece(:,1);
    y(:,1) = piece(:,2);
end

%checks to see if any points of the added piece will be outside of the
%bounds of the given board.
function status = out_of_bounds(point, piece, size)
    x_locations = piece(:,1)+point(1,1);
    x_status = any(x_locations > size) || any(x_locations <= 0);
    y_locations = piece(:,2)+point(1,2);
    y_status = any(y_locations > size) || any(y_locations <= 0);

    if (y_status > 0 || x_status > 0)
        status = 1;
    else
        status = 0;
    end
end

%checks to see if the new point overlaps any point already filled on the
%board
function status = already_filled(point, piece, board)
    status = 0;
    for i = 1:length(piece)
        x_p = piece(i,1)+point(1,1);
        y_p = piece(i,2)+point(1,2);
        piece;
        if board(x_p,y_p) == 0
            status = 1;
            break
        end
    end
end

%writes the given piece to the final board.
function new_board = write_to_board(point, piece, board)
    
    x_locations = piece(:,1)+point(1);
    y_locations = piece(:,2)+point(2);

    for i = 1:length(x_locations)
        board(x_locations(i),y_locations(i)) = 0;
    end
    new_board = board;
end

%checks to see if the part overlaps or goes out of bounds.
function status = check_bad_placement(point, piece, board)
    x_locations = piece(:,1)+point(1);
    y_locations = piece(:,2)+point(2);
    oob = out_of_bounds(point, piece,length(board));
    if oob == 1
        status = 1;
        return
    end
    overlap = already_filled(point, piece, board);
    status = overlap;
end

%randomly chooses to invert, rotate, or do both to the piece to add vairety
function new_shape = alter_shape(piece)
    new_shape = piece;
    if (rand > 0.5)
        new_shape = invert_piece(piece);
    end
    if (rand > 0.5)
        new_shape = rotate_piece(piece);
    end
end