function [final_path,outputArg2] = rrt(robot,world,min_states,max_states,iterations)
    %RRT Summary of this function goes here
    %   Detailed explanation goes here
    %init lists
    number_of_nearest = 3;
    list_of_points = [iterations,length(min_states)];
    G = graph();
    for k = 1:iterations
        %create random point
        rand_config = gen_point(min_states,max_states);
        %check for validity, if so start search for nearest neighbors
        if (check_state_validity(robot, world, rand_config) == false)
            continue
        end
        %search for nearest neighbors
        [idxs, near_neighbors] = find_nearest_points(list_of_points, rand_config, number_of_nearest);
        for i = 1:length(idxs)
            if (check_path_validity(robot,world,rand_config,list_of_points(idx(i))) == false)
                continue
            else
                list_of_points(k) = rand_config;
                %add graph node
                graph.addnode(idxs(i));
                %add graph edge between current neighbor index, and new
                %point index
                graph.addedge(idxs(i),k);
                break;
            end
        end
    end
end

%generates a random configuration between the given limits. 
function p = gen_point(min_states, max_states)
    if isrow(min_states)
        min_states = min_states';
    end
    if isrow(max_states)
        max_states = max_states';
    end
    p = min_states + (max_states-min_states).*rand(length(min_states),1);
end

%checks to see if the given configuration does not run into any obstacles.
function isvalid = check_state_validity(robot,world,config)
    is_collision = robot.check_collision(world,config);
    isvalid = ~is_collision;
end

%creates a linear interpolation between the two points and checks the
%validity of the points along the path.
function valid_path = check_path_validity(robot,world,start_config,end_config)
    interpolation_number = 5;
    %create interpolation between two points
    x = linspace(start_config(1),end_config(1), interpolation_number);
    y = linspace(start_config(2),end_config(2), interpolation_number);
    z = linspace(start_config(3),end_config(3), interpolation_number);
    theta_x = linspace(start_config(4),end_config(4), interpolation_number);
    theta_y = linspace(start_config(5),end_config(5), interpolation_number);
    theta_z = linspace(start_config(6),end_config(6), interpolation_number);
    new_configs = [x;y;z;theta_x;theta_y;theta_z];
    interp_length = length(x);
    %iterates through new interpolation, and breaks if there is any
    %collision found. 
    for i = 1:interp_length
        curr_config = new_configs(:,i);
        valid_state = check_state_validity(robot,world,curr_config);
        if (valid_state == false)
            break;
        end
        valid_path = valid_state;
    end
end

% finds the n nearest points, given the list of points
function [indexs, nearest_points] = find_nearest_points(point_list,new_point,number_of_points)
    %might also add a range search here
    %add sort here for closest first maybe
    mdl = KDTreeSearcher(point_list);
    Idxs = knnsearch(mdl,new_point,'Distance','euclidean','K',number_of_points);
    nearest_points = point_list(Idxs);
    indexs = Idxs;
end