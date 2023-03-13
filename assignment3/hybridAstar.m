classdef hybridAstar
    properties
        timescale = [0 0.6];
        inputs = [1 pi/2; 1 pi/6; 1 pi/12; 1 0;
            1 -pi/2; 1 -pi/6; 1 -pi/12; 1 0;
            -1 pi/2; -1 pi/6; -1 pi/12; -1 0;
            -1 -pi/2; -1 -pi/6; -1 -pi/12; -1 0];

    end

    methods
        function obj = hybridAstar()
        end

        function [path, control_outputs, searched] = search(obj, map, kinematics, start_point, end_point)
        %HYBRIDASTAR Custom implementation of hybrid A* which takes into account
        %the different kinematic models presented for this assignment
        %points are going to be (X,Y,Heading)
        %create structure for storing each points data
        h = 'h';
        f = 'f';
        g = 'g';
        prev = 'previous_point';
        s_a = 'steering_angle';
        v = 'lin_vel';
        w = 'angular_input';
        s = struct(prev,'0  0  0',h,inf,g,inf,f,inf,s_a,0,v,0,w,0);
        
        %coordinate = {num2str(start_point)};
        %value = 0;
        %add starting point to open list
        goal_found = 0;
        open_list = table('Size',[10000 2],'VariableTypes',{'string','double'},'VariableNames',{'coordinate','value'});
        open_list.value = inf(10000,1);
        closed_list = string(zeros(10000,1));
        open_list_index = 1;
        closed_list_index = 1;
        start = num2str(start_point);
        goal = num2str(end_point);

        %start weight list
        ss = s;
        ss.g = 0;
        weights = dictionary(start,ss);
        
        %manually add to the list
        %can change to function call
        open_list(open_list_index,:) = [{num2str(start_point)},0];
        open_list_index = open_list_index + 1;
        
        %visualization
        display_list = zeros(8000,2);
        d_l_index = 1;
        plot(map);

        hold on
        while (isempty(open_list) == 0)
            %this could maybe be one function call
            [open_list_index, lv_node]  = obj.get_least_value_node(open_list, open_list_index);
            [open_list_index, open_list] = obj.remove_node(lv_node, open_list,open_list_index);
            %[closed_list_index,closed_list] = add_node(lv_node, closed_list,closed_list_index, 0);

             cc = str2num(lv_node);
             display_list(d_l_index,:) = cc(1:2);
             %d_l_index = d_l_index + 1;
             scatter(display_list(:,1),display_list(:,2));
             pause(0.1);
        
            if (obj.check_equal(lv_node, goal))
                goal_found = 1;
                break
            else    
                children = obj.find_children(lv_node,kinematics,obj.inputs,map);
                %some children just are repeated and have a bad time. Don't
                %add every child to list?
                for i = 1:length(children)
                    if(children(i) ~= "BAD")
                        child = children(i);
                    %plotter
                       cc = str2num(child);
                        %display_list(d_l_index,:) = cc(1:2);
                        scatter(cc(1),cc(2), 'x');
                        %pause(0.1);
                    current_cost = weights(lv_node).g + obj.euclidean_distance(child, lv_node);
                    %elseif(check_for_node(i, open_list.coordinate))
                    %make sure child is a key, otherwise initialize it
                        if (isKey(weights,child) == 0)
                            weights(child) = s;
                        end
                        if current_cost <= weights(child).g 
                            %update neighbor cells
                            %new_s = update_costs(weights,i,current_cost, euclidean_distance(goal, i));
                             hpp = obj.additional_costs(obj.inputs(i,:),weights(lv_node),child,goal);
                             new_s = s;
                             new_s.previous_point = lv_node;
                             new_s.g = current_cost;
                             new_s.h = obj.euclidean_distance(goal, child)+hpp;
                             new_s.f =  new_s.g + new_s.h;
                             new_s.lin_vel = obj.inputs(i,1);
                             new_s.steering_angle = obj.inputs(i,2);
                             weights(child) = new_s;
                        end
                        if(obj.check_for_node(child, open_list.coordinate) == 0)
                            x = weights(child);
                           [open_list_index, open_list] = obj.add_node(child, open_list,open_list_index,x.f);
                        end
                    end
                end
                [closed_list_index,closed_list] = obj.add_node(lv_node, closed_list, closed_list_index, -1);
            end
        end
        if (goal_found == 1)
            [path,control_outputs] = obj.get_path(weights, start, goal);
        else
            path = 0;
            control_outputs = 0;
        end
        searched = closed_list;
        end
    %%
    %functions
    %sorts table and returns least value node
    % methods (Access = private)
    function [index,x] = get_least_value_node(obj,table,index)
            tsort = sortrows(table,'value');
            x = tsort.coordinate(1);
            %index = index+1;
        end
    
    
        function [ind, t] = remove_node(obj, node, list, index)
            c = list.coordinate;
            found_index = find(c == node);
            list(found_index,:) = [];
            %list(c == node,:) = [];
            t= list;
            ind = index - 1;
        end
        %if weight is negative then don't add weight to list
        %used to differential between open list and closed list
        function [ind, t] = add_node(obj, node, list, index, weight)
            if(weight >= 0)
                list.coordinate(index) = node;
                list.value(index) = weight;
            else
                list(index,:) = node;
            end
            ind = index + 1;
            t = list;
        end
    
        function eqflag = check_equal(obj, node1, node2)
            n1 = round(str2num(node1),1);
            n2 = round(str2num(node2),1);
            eqflag = all(n1 == n2);
            %TODO better equal check
        end
    
        function children =  find_children(obj, node, kinematics, inputs, map)
            children = string(length(inputs));
            %set current position
            kinematics.current_position = str2num(node)';
            %iterate through control inputs to find outputs. 
                for i = 1:length(inputs)
                    [fp,t,ps] = kinematics.integrate_position(inputs(i,1),inputs(i,2),obj.timescale);
                    %check for collision
                    if(kinematics.check_collision(map,ps) == 0)
                        children(i) = num2str(round(fp,2));
                    else
                        children(i) = "BAD";
                    end
                end
            end
    
        %node in string form.
        function check = check_for_node(obj, node, list)
            check = any(list == node);
        end

        %node in string form
        function s = update_costs(obj, dict,node,g,h)
           s = dict(node);
           s.h = h;
           s.g = g;
           s.f = g+h;
    
        end
        %takes in string version of point
        function d = euclidean_distance(obj, node1, node2)
            n1 = str2num(node1); 
            n2 = str2num(node2);
            d = sqrt(sum((n2(1:2)-n1(1:2)).^2));
        end
        %takes in string version of point
        function [path,ctrl_inputs] = get_path(obj, dict, start, goal)
            path = string(zeros(1000,1));
            ctrl_inputs = zeros(1000,2);
            d = dict(goal);
            path(1) = goal;
            ctrl_inputs(1,1) = d.lin_vel;
            ctrl_inputs(1,2) = d.steering_angle;
            curr_point = goal;
            path_index = 2;
            while(curr_point ~= start)
                d = dict(curr_point);
                curr_point = d.previous_point;
                path(path_index) = curr_point;
                ctrl_inputs(path_index,1) = d.lin_vel;
                ctrl_inputs(path_index,2) = d.steering_angle;
                path_index = path_index + 1;
            end
            path = path(1:path_index-1,:);
            ctrl_inputs = ctrl_inputs(1:path_index-1,:);
        end

        %additional cost adders for hybrid approach
        function hpp = additional_costs(obj,input,weight_struct,node,goal)
            hpp = 0;
            %cost adder for going backwards
            if(input(1) < 0)
                hpp = hpp + 0.5;
            end
            %cost adder for changing the turning input
            %cost adder for proper heading comparison.

        end
    end
end