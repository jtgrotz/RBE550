function [path, control_outputs, searched] = hybridAstar(map, kinematics, start_point, end_point)
%HYBRIDASTAR Custom implementation of hybrid A* which takes into account
%the different kinematic models presented for this assignment
%points are going to be (X,Y,Heading)
%create structure for storing each points data
h = 'h'
f = 'f'
g = 'g'
prev = 'previous_point'
s = struct(prev,'0 0',h,inf,g,inf,f,inf);

coordinate = {num2str(start_point)};
value = 0;
%add starting point to open list
open_list = table('Size',[10 2],'VariableTypes',{'string','double'},'VariableNames',{'coordinate','value'});
closed_list = string(zeros(10,1));
open_list_index = 1;
closed_list_index = 1;
weights = dictionary(string([]),[]);
start = num2str(start_point);
goal = num2str(end_point);

open_list(open_list_index,:) = [{num2str(start_point)},0];
open_list_index = open_list_index + 1;

while (isempty(open_list) == 0)
    [open_list_index lv_node]  = get_least_value_node(open_list, open_list_index);
    [open_list_index, open_list] = remove_node(lv_node, open_list);
    %[closed_list_index,closed_list] = add_node(lv_node, closed_list,closed_list_index, 0);

    if (check_equal(lv_node, end_point))
        break
    else
        children = find_children(lv_node,kinematics,map);
        for i = 1:length(children)
            %if(check_for_node(i, closed_list) == 0)
            current_cost = weights(lv_node).g + euclidean_distance(i, lv_node);
            %elseif(check_for_node(i, open_list.coordinate))
                if current_cost < weights(i).g 
                    %update neighbor cells
                    %new_s = update_costs(weights,i,current_cost, euclidean_distance(goal, i));
                     new_s = s;
                     new_s.previous_point = lv_node;
                     new_s.g = current_cost;
                     new_s.h = euclidean_distance(goal, i);
                     new_s.f =  new_s.g + new_s.h;
                     weights(i) = new_s;
                end
            if(check_for_node(i, open_list.coordinate) == 0)
               [open_list_index, open_list] = add_node(i, open_list);
            end
        end
        
    [closed_list_index,closed_list] = add_node(lv_node, closed_list,closed_list_index, 0);
    end
end

path = get_path(weights, start, goal);

end
%%
%functions
%sorts table and returns least value node
    function x = get_least_value_node(t)
        tsort = sortrows(table,'value');
        x = tsort.coordinates(1)
        x = x{1}
    end


    function [ind, t] = remove_node(node, list, index)
        c = list.coordinates;
        %found_index = find(c == node);
        %list(found_index,:) = [];
        list(c == node,:) = [];
        t= list;
        ind = index - 1;
    end

    function [ind, t] = add_node(node, list, index, weight)
        list(index,:) = [node, weight];
        ind = index + 1;
        t = list;
    end

    function eqflag = check_equal(node1, node2)
        n1 = round(str2num(node1),1);
        n2 = round(str2num(node2),1);
        eqflag = all(n1 == n2);
    end

    function find_children(node, kinematics, map)
    end

    function check = check_for_node(node, list)
        check = any(list == node);
    end

    function s = update_costs(dict,node,g,h)
       s = dict(node);
       s.h = h;
       s.g = g;
       s.f = g+h;

    end

    function d = euclidean_distance(node1, node2)
        n1 = num2str(node1); 
        n2 = num2str(node2);
        d = sqrt(sum((n2-n1).^2));
    end

    function path = get_path(dict, start, goal)

    end