function [path, control_outputs, searched] = hybridAstar(map, kinematics, start_point, end_point)
%HYBRIDASTAR Custom implementation of hybrid A* which takes into account
%the different kinematic models presented for this assignment
%points are going to be (X,Y,Heading)
open_list = {};
closed_list = {};
weights = dictionary(string([]),[]);

open_list{end+1} = num2str(start_point);

while (isempty(open_list) == 0)
    lv_node  = get_least_value_node();
    remove_node(lv_node, list);
    add_node(lv_node, closed_list);

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

path = get_path();

end

