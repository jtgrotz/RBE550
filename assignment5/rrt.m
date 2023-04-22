function [final_path,outputArg2] = rrt(robot,world,min_states,max_states,iterations)
    %RRT Summary of this function goes here
    %   Detailed explanation goes here
    %init lists
    number_of_nearest = 3;
    list_of_points
    graph = graph()
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
                graph.addnode()
                %add graph edge
                graph.addedge()
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
end

%creates a linear interpolation between the two points and checks the
%validity of the points along the path.
function valid_path = check_path_validity(robot,world,start_config,end_config)
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