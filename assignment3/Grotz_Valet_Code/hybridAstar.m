classdef hybridAstar
    properties
        timescale = [0 1];
        inputs = [];
        modified_inputs;
        heading_tolerance = 10; %degrees
        position_tolerance = 0.3;%meters

    end

    methods
        function obj = hybridAstar()
        end

        function [path, control_outputs, searched] = search(obj, map, kinematics, start_point, end_point)
        %HYBRIDASTAR Custom implementation of hybrid A* which takes into account
        %the different kinematic models presented for this assignment
        %points are going to be (X,Y,Heading)
        %create structure for storing each points data
        if(length(start_point) > 3)
            trailer_flag = 1;
        else
            trailer_flag = 0;
        end
        h = 'h';
        f = 'f';
        g = 'g';
        prev = 'previous_point';
        s_a = 'steering_angle';
        v = 'lin_vel';
        w = 'angular_input';
        s = struct(prev,'0  0  0',h,inf,g,inf,f,inf,s_a,0,v,0,w,0);
        obj.modified_inputs = obj.inputs;
        debug_counter = 0;
        
        %add starting point to open list
        goal_found = 0;
        open_list = table('Size',[80000 2],'VariableTypes',{'string','double'},'VariableNames',{'coordinate','value'});
        open_list.value = inf(80000,1);
        closed_list = string(zeros(80000,1));
        open_list_index = 1;
        closed_list_index = 1;
        start = num2str(start_point);
        goal = num2str(end_point);
        %baseline metric for distance to goal
        total_e_distance = obj.euclidean_distance(start,goal);

        %start weight list
        ss = s;
        ss.g = 0;
        weights = dictionary(start,ss);
        
        %manually add to the list
        %can change to function call
        open_list(open_list_index,:) = [{num2str(start_point)},0];
        open_list_index = open_list_index + 1;
        
        %visualization code
        display_list = zeros(8000,2);
        d_l_index = 1;
        vis_index = 60;
        f = figure;
        
        while (isempty(open_list) == 0 || open_list_index == 0)
            debug_counter = debug_counter + 1;
            %disp(debug_counter);

            %grab the least value node, remove it from the open list, and
            %add to the closed list.
            [open_list_index, lv_node]  = obj.get_least_value_node(open_list, open_list_index);
            [open_list_index, open_list] = obj.remove_node(lv_node, open_list,open_list_index);
            [closed_list_index,closed_list] = obj.add_node(lv_node, closed_list, closed_list_index, -1);

             
            %code for visualization
            cc = str2num(lv_node);
            display_list(d_l_index,:) = cc(1:2);
            d_l_index = d_l_index + 1;
            if (trailer_flag == 1)
            cc_x_t = cc(1)-kinematics.hitch_length*cos(-cc(4));
            cc_y_t = cc(2)-kinematics.hitch_length*sin(-cc(4));
            display_list(d_l_index,:) = [cc_x_t,cc_y_t];
            d_l_index = d_l_index + 1;
            end

            %plotting visualization
            %only plots every 60 iterations
            vis_index = vis_index +1;
            if (vis_index > 60)
                 plot(map);
                 hold on
                 scatter(display_list(:,1),display_list(:,2));
                 hold off
                 pause(0.1);
                 vis_index = 0;
            end
        
                %check if goal was found
                %checks if within a certain tolerance
            if (obj.check_equal(lv_node, goal))
                ss = s;
                ss.previous_point = lv_node;
                weights(goal) = ss;
                goal_found = 1;
                break
                %stops the run after so many iterations
            elseif (debug_counter >= 20000)
                ss = s;
                ss.previous_point = lv_node;
                weights(goal) = ss;
                goal_found = 1;
                break
            else
                 %adjust movement primatives based on proximity to the
                 %goal
                 %might change to linear scale velocity as we get closer to
                 %the goal
                 if (obj.euclidean_distance(lv_node,goal) <= total_e_distance *0.2)
                     obj.modified_inputs(:,1) = obj.modified_inputs(:,1)*0.5;
                 else
                     obj.modified_inputs = obj.inputs;
                 end

                 %finds children of least value node.
                 %inegrated over set inputs.
                children = obj.find_children(lv_node,kinematics,obj.modified_inputs,map);
                for i = 1:length(children)
                    %if the node isn't bad and it's not closed, then
                    %continue
                    child = children(i);
                    if(child == "BAD")
                        continue
                    elseif(obj.check_for_node(child,closed_list) == 1)
                        continue
                    else

                   %adds custom movement cost.
                    current_cost = weights(lv_node).g + obj.movement_cost(lv_node,child,i);
                    %make sure child is a key, otherwise initialize it
                        if (isKey(weights,child) == 0)
                            weights(child) = s;
                        end
                        if current_cost <= weights(child).g 
                            %update neighbor cells
                             new_s = s;
                             new_s.previous_point = lv_node;
                             new_s.g = current_cost;
                             new_s.h = obj.euclidean_distance(goal, child)+obj.heading_error(goal,child);
                             new_s.f =  new_s.g + new_s.h;
                             new_s.lin_vel = obj.modified_inputs(i,1);
                             new_s.steering_angle = obj.modified_inputs(i,2);
                             weights(child) = new_s;
                        end
                        %if child is not in the list of open nodes then add
                        %if child is in the list, then update cost.
                        if(obj.check_for_node(child, open_list.coordinate) == 0)
                            x = weights(child);
                           [open_list_index, open_list] = obj.add_node(child, open_list,open_list_index,x.f);
                        else
                            %remove node then readd with new weight
                            [open_list_index, open_list] = obj.remove_node(child, open_list,open_list_index);
                            [open_list_index, open_list] = obj.add_node(child, open_list,open_list_index,x.f);

                        end
                    end
                end
            end
        end
        if (goal_found == 1)
            [path,control_outputs] = obj.get_path(weights, start, lv_node);
        else
            path = 0;
            control_outputs = 0;
        end
        searched = closed_list(1:closed_list_index);
        end

        %additional functions
        %sorts table and returns least value node
        function [index,x] = get_least_value_node(obj,table,index)
            tsort = sortrows(table,'value');
            x = tsort.coordinate(1);
        end
    
        %removes node from given index, and decrements index counter
        function [ind, t] = remove_node(obj, node, list, index)
            c = list.coordinate;
            found_index = find(c == node);
            list(found_index,:) = [];
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
    
        %checks if two nodes are equal within a defined tolerance
        function eqflag = check_equal(obj, node1, node2)
            n1 =str2num(node1);
            n2 =str2num(node2);
            %eqflag = all(n1 == n2);
            %TODO better equal check
            d = obj.euclidean_distance(node1,node2);
            h1a = n1(3);
            h2a = n2(3);
            ha_error = mod(((h1a-h2a)*180/pi),360);
            if(length(n1) == 4)
                h1b = n1(4);
                h2b = n2(4);
                hb_error = mod(((h1b-h2b)*180/pi),360);
            else
                hb_error = 0;
            end
            if (d <= obj.position_tolerance && ha_error <= obj.heading_tolerance && hb_error <= obj.heading_tolerance)
                eqflag = 1;
            else
                eqflag = 0;
            end
        end
    
        %finds children of the current node through kinematic integration
        function children =  find_children(obj, node, kinematics, inputs, map)
            children = string(length(inputs));
            %set current position
            kinematics.current_position = str2num(node)';
            %iterate through control inputs to find outputs. 
                for i = 1:length(inputs)
                    [fp,t,ps] = kinematics.integrate_position(inputs(i,1),inputs(i,2),obj.timescale);
                    %convert points from rad to degrees
                    ps(:,3:end) = ps(:,3:end)*180/pi;
                    %check for collision
                    if(kinematics.check_collision(map,ps) == 0)
                        children(i) = num2str([round(fp(1:2),1),round(fp(3:end),2)]);
                    else
                        children(i) = "BAD";
                    end
                end
            end
    
        
        %node in string form.
        %checks for a node in a list
        function check = check_for_node(obj, node, list)
            check = any(list == node);
        end

        %node in string form
        %updates a cost for the node in a the dictionary
        function s = update_costs(obj, dict,node,g,h)
           s = dict(node);
           s.h = h;
           s.g = g;
           s.f = g+h;
    
        end

        %takes in string version of point
        %finds the euclidean distance to the goal.
        function d = euclidean_distance(obj, node1, node2)
            n1 = str2num(node1); 
            n2 = str2num(node2);
            d = sqrt(sum((n2(1:2)-n1(1:2)).^2));
        end

        %takes in string version of point
        %iterates back through the previous points to find path to the
        %goal.
        function [path,ctrl_inputs] = get_path(obj, dict, start, goal)
            point = str2num(goal);
            if (length(point) > 3)
                path = zeros(1000,4);
            else
                path = zeros(1000,3);
            end
            ctrl_inputs = zeros(1000,2);
            %d = dict(goal);
            path(1,:) = str2num(goal);
            %ctrl_inputs(1,1) = d.lin_vel;
            %ctrl_inputs(1,2) = d.steering_angle;
            curr_point = goal;
            path_index = 2;
            while(any(str2num(curr_point) ~= str2num(start)))
                d = dict(curr_point);
                curr_point = d.previous_point;
                path(path_index,:) = str2num(curr_point);
                ctrl_inputs(path_index,1) = d.lin_vel;
                ctrl_inputs(path_index,2) = d.steering_angle;
                path_index = path_index + 1;
            end
            path = flip(path(1:path_index-1,:),1);
            ctrl_inputs = flip(ctrl_inputs(1:path_index-1,:),1);
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

        %additional cost to show robot doesn't have proper heading.
        function e = heading_error(obj,node1,node2)
            n1 =str2num(node1);
            n2 =str2num(node2);
            h1a = n1(3);
            h2a = n2(3);
            ha_error = mod(abs((h1a-h2a)*180/pi),360);
            if(length(n1) == 4)
                h1b = n1(4);
                h2b = n2(4);
                hb_error = mod(abs((h1b-h2b)*180/pi),360);
            else
                 hb_error = 0;
            end
            e = (ha_error+hb_error)/(360*2);
        end

        %determines movement cost for motion primative
        %accounts for linear and angular
        %punishes backwards movement, or not linear movement
        function c = movement_cost(obj,node1,node2,index)
            lin_delta = obj.modified_inputs(index,1);
            if (lin_delta < 0)
                lin_delta = abs(lin_delta)+2;
            end
            angle_delta = obj.heading_error(node1,node2);
            if (lin_delta == 0)
                %adds a cost adder for not moving forward.
                angle_delta = angle_delta+2;
            end
            c = lin_delta + angle_delta;
        end
    end
end