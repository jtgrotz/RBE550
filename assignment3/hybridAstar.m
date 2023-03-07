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
closed_list = {};
open_list_index = 1;
closed_list_index = 1;
weights = dictionary(string([]),[]);

open_list(end+1,:) = [{num2str(start_point)},0];

while (isempty(open_list) == 0)
    [open_list_index lv_node]  = get_least_value_node(open_list, open_list_index);
    [open_list_index, open_list] = remove_node(lv_node, open_list);
    [closed_list_index,closed_list] = add_node(lv_node, closed_list,closed_list_index, 0);

    if (check_equal(lv_node, end_point))
        break
    else
        children = find_children(lv_node,kinematics,map);
        for i = 1:length(children)
            if(check_for_node(children, closed_list))
                %do nothing
            else
                update_costs()
            end
        end 

    end


end

path = get_path();

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

    function check_for_node(node, list)
    end

    function update_costs(dict,node,g,h)
        

    end

    function d = euclidean_distance(node1, node2)
        n1 = num2str(node1); 
        n2 = num2str(node2);
        d = sqrt(sum((n2-n1).^2));
    end

    function path = get_path()
    end