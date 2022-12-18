function [opt_st_path, closed_list, open_list, planning_para] = ...
    HeuristicSearchFunc(speed_para, ego_init_state, ...
    obstacles_info, st_graph)
    open_list = [];
    closed_list = [];
    
    planning_para = SetPlanningCfgFunc(st_graph,speed_para);
    
    init_node = CalculateInitNodeFunc( ...
            planning_para, st_graph, ego_init_state);

    open_list = [open_list; init_node];
    
    aaa = 0;
    
    while ~isempty(open_list)
        [~, idx_parent_node] = min(open_list(:, 4));
        parent_node = open_list(idx_parent_node, :);
        
        if parent_node(5) == st_graph.total_time || ...
           parent_node(6) == st_graph.total_s
            break;
        end

        open_list(idx_parent_node, :) = [];    
        closed_list = [closed_list; parent_node];

        child_nodes = ...
                CalculateChildNodesFunc(parent_node, st_graph, ...
                                        planning_para, speed_para);

        while ~isempty(child_nodes)
            child_node = child_nodes(1, :);
            child_nodes(1, :) = [];
            
            if ismember(child_node(1 : 2), closed_list(:, 1 : 2), 'rows')
                continue;
            end

            [in_open_list_flag, idx] = ...
                    ismember(child_node(1 : 2), ...
                             open_list(:, 1 : 2), 'rows');

            [f_cost, g_cost, h_cost, obs_cost, edg_cost] = ...
                    CalculateTotalCost(parent_node, child_node, speed_para, ...
                                       planning_para, st_graph, ...
                                       obstacles_info);

            child_node(3 : 4) = [g_cost, f_cost];
            child_node(11 : 12) = [h_cost, obs_cost];
            child_node(14) = edg_cost;

            if in_open_list_flag
                if f_cost < open_list(idx, 4)
                    open_list(idx, :) = child_node;
                end
            else
                open_list = [open_list; child_node];
            end
        end
    end
    
    if isempty(open_list) || parent_node(4) == Inf
        disp('[warning]there is no path solution!');
       
        opt_st_path = [];

        return;
    end

    opt_st_path = ...
            CalculateOptimalSTPath(closed_list, parent_node, ... % s_dense && s_sparse
                                   planning_para,st_graph);
end

function [planning_para] = SetPlanningCfgFunc(st_graph, speed_para)
    planning_para.w_edge = 1.0;
    
    planning_para.w_exceed_speed  = 2000.0; % max_val = Inf || 1
    planning_para.w_deceed_speed = 100.0; % max_val = 251
    planning_para.w_cruise_speed = 3000.0; % max_val =1001
    planning_para.w_keep_clean_speed = 2.0; % val = 50 + w * 0 - 5^2
    planning_para.keep_clean_low_speed_penalty = 50.0; % val = 50
    
    planning_para.w_lon_acc = 100; % max_val = 1
    planning_para.w_lon_deacc = 100; % max_val = 1
    planning_para.w_lat_acc = 1; % max_val = 1
    
    planning_para.w_positive_jerk = 1; % max_val = 1
    planning_para.w_negative_jerk = 1; % max_val = 1
    
    planning_para.w_spatial = 150; % 于CalculateObstacleFunc()中动态调整权重值 % 与障碍物数量正相关 0 - 150； 1 - 150*  550；2 - 150 * 50 * 10
    
    planning_para.w_obstacle = 6000.0; % 1000000.0
    planning_para.s_safe_follow = 20.0; % 取值与本车速度、相对速度相关
    planning_para.s_safe_overtake = 20.0;
    planning_para.default_safe_dis_penalty = 0.0;
    planning_para.max_lon_control_err = 2.5 + 1.0; % 控制误差：2.5m; 后端优化器位置偏差最大值：2.0m
    
    planning_para.speed_overrun_ratio = 0.1;
    planning_para.lat_acc_max = 2.5;
    
    planning_para.acc_max = 4.0;
    planning_para.acc_min = -6.0;
    planning_para.jerk_max = 5.0;
    planning_para.jerk_min = -10.0;
    
    planning_para.delta_t = st_graph.dt;
    
    planning_para.opt_path_point_interval = 0.1; % time interval
end

function [init_node] = CalculateInitNodeFunc(...
        planning_para, st_graph, ego_init_state)
    idx_t = 1;
    idx_s = length(st_graph.s_dense);
    g_cost =0;
    f_cost = ...
            planning_para.w_spatial * st_graph.total_s / ...
            ego_init_state.init_speed / st_graph.dt;
    t = 0;
    s = 0;
    speed = ego_init_state.init_speed;
    acc = ego_init_state.init_acc;
    idx_t_parent = Inf;
    idx_s_parent = Inf;
    h_cost = ...
            planning_para.w_spatial * st_graph.total_s / ...
            ego_init_state.init_speed / st_graph.dt;
    obs_cost = 0;
    edg_cost = 0;
    
    kappa = st_graph.kappa_dense(end);

    init_node = ...
            [idx_t, idx_s, g_cost, f_cost, t, s, ...
            speed, acc, idx_t_parent, idx_s_parent, ...
            h_cost, obs_cost, kappa, edg_cost];
end

function [child_nodes] = CalculateChildNodesFunc( ...
        parent_node,st_graph,planning_para,speed_para)
    child_nodes = [];
    
    time_margin = 0.000001;
    dt = st_graph.dt + time_margin;
    dt_square = dt^2;
    dt_cube = dt^3;
    
    % s1 = s0 + vo * dt + 1/2 * a0 * dt^2 + 1 / 6 * jerk * dt_cube;
    s_min = ...
            parent_node(6) + parent_node(7) * dt + ...
            1 / 2 * parent_node(8) * dt_square + ...
            1 / 6 * planning_para.jerk_min * dt_cube;
    s_max = ...
            parent_node(6) + parent_node(7) * dt + ...
            1 / 2 * parent_node(8) * dt_square + ...
            1 / 6 * planning_para.jerk_max * dt_cube;
        
    idx_t_dividing = st_graph.t_dividing / st_graph.dt + 1;
    child_node_idx_t = parent_node(1)+1;
    
    if child_node_idx_t <= idx_t_dividing
        for i = parent_node(2): -1 : 1
            if st_graph.s_dense(i) > s_min && st_graph.s_dense(i) <= s_max
                s0 = parent_node(6);
                v0 = parent_node(7);
                a0 = parent_node(8);

                s1 = st_graph.s_dense(i);
                
                jerk = ...
                        (s1 - s0 - v0 * dt - 0.5 * a0 * dt_square) * ...
                        6 / dt_cube;

                a = a0 + jerk * dt;

                v = v0 + a0 * dt + 0.5 * jerk * dt_square;

                speed_overrun_ratio = planning_para.speed_overrun_ratio;
                speed_limit = ...
                        speed_para.speed_limit * (1 + speed_overrun_ratio); % speed_limit应该取自st_graphd的speed_bound

                kappa = st_graph.kappa_dense(i); % 曲率限速放在上游更合适        
                centrifugal_acc = v^2 * kappa;

                if v < 0 || ...
                   v > speed_limit || ...
                   a > planning_para.acc_max || ...
                   a < planning_para.acc_min || ...
                   centrifugal_acc > planning_para.lat_acc_max
                    continue;
                end

                idx_t = parent_node(1) + 1;
                idx_s = i;
                g_cost = Inf;
                f_cost = Inf;
                t = st_graph.t(idx_t);
                s = st_graph.s_dense(i);
                speed = v;
                acc = a;
                idx_t_parent = parent_node(1);
                idx_s_parent = parent_node(2);
                h_cost = Inf;
                obs_cost = Inf;
                edg_cost = Inf;
                
                if idx_t == 8 && idx_s == 504
                    test = 1;
                end

                child_node = ...
                        [idx_t, idx_s,  g_cost, f_cost, t, s, ...
                        speed, acc,  idx_t_parent, idx_s_parent, ...
                        h_cost, obs_cost, kappa, edg_cost];

                child_nodes = [child_nodes; child_node];
            elseif st_graph.s_dense(i) > s_max;
                break;
            end
        end
    else
        for i = length(st_graph.s_sparse) : -1 : 1
            if st_graph.s_sparse(i) > s_min && ...
               st_graph.s_sparse(i) <= s_max
                s0 = parent_node(6);
                v0 = parent_node(7);
                a0 = parent_node(8);

                s1 = st_graph.s_sparse(i);

                jerk = ...
                        (s1 - s0 - v0 * dt - 1 / 2 * a0 * dt_square) * ...
                        6 / dt_cube;

                a = a0 + jerk * dt;

                v = v0 + a0 * dt + 0.5 * jerk * dt_square;

                speed_overrun_ratio = planning_para.speed_overrun_ratio;
                speed_limit=  ...
                        speed_para.speed_limit * (1 + speed_overrun_ratio);

                kappa = st_graph.kappa_sparse(i);
                centrifugal_acc = v^2 * kappa;

                if v < 0 || ...
                   v > speed_limit || ...
                   a > planning_para.acc_max || ...
                   a < planning_para.acc_min || ...
                   centrifugal_acc > planning_para.lat_acc_max
                    continue;
                end

                idx_t = parent_node(1) + 1;
                idx_s = i;
                g_cost = Inf;
                f_cost = Inf;
                t = st_graph.t(idx_t);
                s = st_graph.s_sparse(i);
                speed = v;
                acc = a;
                idx_t_parent = parent_node(1);
                idx_s_parent = parent_node(2);
                h_cost = Inf;
                obs_cost = Inf;
                edg_cost = Inf;
                
                if idx_t == 9 && idx_s == 456
                    test = 1;
                end

                child_node = ...
                        [idx_t, idx_s, g_cost, f_cost, t, s, ...
                        speed, acc, idx_t_parent, idx_s_parent, ...
                        h_cost, obs_cost, kappa, edg_cost];

                child_nodes = [child_nodes; child_node];
            elseif st_graph.s_sparse(i) > s_max
                break;
            end
        end
    end
end

function [f_cost, g_cost, h_cost, obs_cost, edg_cost] = CalculateTotalCost(...
        parent_node, child_node, speed_para, ...
        planning_para, st_graph, obstacles_info)
    [g_cost, edg_cost, obs_cost] =  ...
            CalculateGCostFunc(parent_node, child_node, speed_para, ...
            planning_para, st_graph, obstacles_info);

    h_cost = ...
            (st_graph.total_s - child_node(6)) / child_node(7) / ...
            st_graph.dt * planning_para.w_spatial;

    f_cost = g_cost + h_cost;
end

function [g_cost, edg_cost, obs_cost] = CalculateGCostFunc( ...
        parent_node, child_node, speed_para, ...
        planning_para, st_graph, obstacles_info)
    edg_cost = ...
            CalculateEdgeCost(parent_node, child_node, speed_para, ...
            planning_para, st_graph);
        
    if edg_cost == Inf
        g_cost = Inf;
        edg_cost = Inf;
        obs_cost = Inf;
        
        return;
    end

    obs_cost = ...
            CalculateObstacleCostFunc(child_node, planning_para, ...
            obstacles_info, parent_node);
        
    g_cost = parent_node(3) + edg_cost + obs_cost;
end

function [edg_cost] = CalculateEdgeCost( ...
        parent_node, child_node, speed_para, planning_para, st_graph)
    [speed_cost, speed] = ...
            CalculateSpeedCostFunc(parent_node, child_node, ...
                                   speed_para, planning_para);
    
    if speed_cost == Inf
        edg_cost = Inf;
        return;
    end

    acc_cost = ....
            CalculateAccCostFunc(parent_node, child_node, ...
                                 planning_para, speed);

    if acc_cost == Inf
        edg_cost = Inf;
        return;
    end

    jerk_cost = ...
            CalculateJerkCostFunc(parent_node, child_node, ...
                                  planning_para, st_graph);
    
	edg_cost = (speed_cost + acc_cost + jerk_cost) * planning_para.delta_t;
end

function [speed_cost, speed] = CalculateSpeedCostFunc( ...
        parent_node, child_node, speed_para, planning_para)
    speed_overrun_ratio = planning_para.speed_overrun_ratio;
    
    speed_limit = ...
            speed_para.speed_limit * ...
            (1 + speed_overrun_ratio);

    speed_max = max(child_node(7), parent_node(7));
    speed_min = min(child_node(7), parent_node(7));

    v0 = parent_node(7);
    a0 = parent_node(8);
    jerk = (child_node(8) - parent_node(8)) / planning_para.delta_t;
    
    step_max = 0.3;
    
    step = planning_para.delta_t / ceil(planning_para.delta_t / step_max);
    
    t = step;

    while t < planning_para.delta_t
        v = v0 + a0 * t + 0.5 * jerk*t^2;
    
        if v > speed_limit || v < 0
            speed_cost = Inf;
            speed = Inf;
            return;
        end
        
        speed_max = max(v, speed_max);
        speed_min = min(v, speed_min);
        
        t = t + step;
    end
    
    w_exceed_speed = planning_para.w_exceed_speed;
    w_deceed_speed = planning_para.w_deceed_speed;
    w_cruise_speed = planning_para.w_cruise_speed;
    w_keep_clean_speed = planning_para.w_keep_clean_speed;
    
    exceed_speed_cost = 0;
    deceed_speed_cost = 0;
    
    speed_limit = speed_para.speed_limit;
    
    diff_speed = speed_max - speed_limit;
    if diff_speed > 0
        scale_factor = 1 / (speed_limit * speed_overrun_ratio);
        diff_speed = diff_speed * scale_factor;

        exceed_speed_cost = ...
                w_exceed_speed * diff_speed^2;
    end  

    diff_speed = speed_min - speed_limit;
    if diff_speed < 0 && ~speed_para.cruise_state_flag
        deceed_speed_ratio = 0.2;

        scale_factor = 1 / (speed_limit * deceed_speed_ratio);

        diff_speed = diff_speed * scale_factor;

        deceed_speed_cost = w_deceed_speed * diff_speed^2;
    end

    cruise_cost = 0;
    
    if speed_para.cruise_state_flag
%         diff_speed = speed - speed_para.cruise_speed;
        diff_speed = ... % 偏向该计算方式，待测试
                max(abs(speed_max - speed_para.cruise_speed), ...
                abs(speed_para.cruise_speed - speed_min));
        
        cruise_speed_err_ratio = 0.1;

        scale_factor = ...
                1 / (speed_para.cruise_speed * cruise_speed_err_ratio);
            
        diff_speed = diff_speed * scale_factor;
        
        cruise_cost = w_cruise_speed * diff_speed^2;
        
        if speed_para.cruise_speed > speed_para.speed_limit
            disp('[warning]cruise_speed is bigger than speed_limit');
        end
    end

    keep_clean_cost = 0;

    if speed_para.keep_clean_flag && ...
       speed_min < speed_para.keep_clean - min_speed
        diff_speed = -speed_min + speed_para.keep_clean_min_speed;

        keep_clean_deceed_ratio = 0.2;

        scale_factor = ...
                1 / (speed_para.keep_clean_min_speed * ...
                keep_clean_deceed_ratio);

        diff_speed = diff_speed * scale_factor;

        keep_clean_cost = ... % 主要依托常数代价项planning._keep_clean_low_speed_penalty
                w_keep_clean_speed * diff_speed^2 + ...
                planning_para.keep_clean_low_speed_penalty;
    end

    speed = speed_max;

    speed_cost = ...
            exceed_speed_cost + deceed_speed_cost + ...
            cruise_cost + keep_clean_cost;
end

function [acc_cost] = CalculateAccCostFunc( ...
        parent_node, child_node, planning_para, speed)
    lon_acc_cost = ...
            CalculateLongitudeAccFunc(parent_node, child_node, ...
                                      planning_para);
    
    if lon_acc_cost == Inf
        acc_cost = Inf;
        return;
    end

    lat_acc_cost = ...
            CalculateLatitudeAccFunc(parent_node, child_node, ...
            planning_para, speed);

        acc_cost = lon_acc_cost + lat_acc_cost;
end

function [lon_acc_cost] = CalculateLongitudeAccFunc( ...
        parent_node, child_node, planning_para)
    w_lon_acc = planning_para.w_lon_acc;
    w_lon_deacc = planning_para.w_lon_deacc;
    
    lon_acc_lb = planning_para.acc_min;
    lon_acc_ub = planning_para.acc_max;
    
    lon_acc_min = min(child_node(8), parent_node(8));
    lon_acc_max = max(child_node(8), parent_node(8));
    
    if lon_acc_max < 0 % 计算子节点处确保lon_acc不超限，此处可不检测
        scale_factor = 1 / lon_acc_lb;
        lon_acc_cost = ...
                w_lon_deacc * ((lon_acc_max * scale_factor)^2 + ...
                (lon_acc_min * scale_factor)^2) / 2;
    elseif lon_acc_min > 0
        scale_factor = 1 / lon_acc_ub;
        lon_acc_cost = ...
                w_lon_acc * ((lon_acc_max * scale_factor)^2 + ...
                (lon_acc_min * scale_factor)^2) / 2;
    else
        scale_factor = 1 / lon_acc_lb;
        deacc_cost = w_lon_deacc * (lon_acc_min * scale_factor)^2;
        
        scale_factor = 1 / lon_acc_ub;
        acc_cost = w_lon_acc * (lon_acc_max * scale_factor)^2;
        
        acc_time_proportion = lon_acc_max / (lon_acc_max - lon_acc_min);
        lon_acc_cost = ...
                (deacc_cost * (1 - acc_time_proportion) + ...
                acc_cost * acc_time_proportion) / 2;
    end
end

function [lat_acc_cost] = CalculateLatitudeAccFunc( ...
        parent_node, child_node, planning_para, speed)
    kappa = max(abs(child_node(13)), abs(parent_node(13)));

    lat_acc = speed^2 * kappa;
    
    safe_lat_acc = planning_para.lat_acc_max * 0.6;

    if lat_acc > planning_para.lat_acc_max
        lat_acc_cost = Inf;
    elseif abs(lat_acc) < safe_lat_acc
        lat_acc_cost = 0;
    else
        scale_factor = 1 / planning_para.lat_acc_max;
        lat_acc = lat_acc * scale_factor;
        
        lat_acc_cost = planning_para.w_lat_acc * lat_acc^2;
    end
end

function [jerk_cost] = CalculateJerkCostFunc( ...
        parent_node, child_node, planning_para, st_graph)
    jerk = (child_node(8) - parent_node(8)) / st_graph.dt;
    
    if jerk > 0
        scale_factor = 1 / planning_para.jerk_max;
        jerk = jerk * scale_factor;
        
        jerk_cost = ...
                planning_para.w_positive_jerk * jerk^2;
    else
        scale_factor = 1 / planning_para.jerk_min;
        jerk = jerk * scale_factor;

        jerk_cost = ...
                planning_para.w_negative_jerk * jerk^2;
    end
end

function [obs_cost] = CalculateObstacleCostFunc( ...
        child_node, planning_para, obstacles_info, parent_node)
    obs_cost = 0;
    
    if isempty(obstacles_info(1).st_overlap)
        return;
    end
    
    for i = 1 : 1 :  length(obstacles_info)
        if obstacles_info(i).obs_type ~= 1
           continue;
        end
        
        s0 = parent_node(6);
        v0 = parent_node(7);
        a0 = parent_node(8);
        jerk =  (child_node(8) - parent_node(8))/  planning_para.delta_t;
        
        step = planning_para.delta_t / 5;
        
        t = step;
        
        while t <= planning_para.delta_t
            ego_t = t + parent_node(5);

            time_margin = 0.001;
            
            if ego_t < obstacles_info(i).st_overlap(1, 1) - time_margin || ...
               ego_t > obstacles_info(i).st_overlap(end, 1) + time_margin
                t = t + step;
                continue;
            end

            s = s0 + v0 * t + 0.5 * a0 * t^2 + 1 /  6 * jerk * t^3;
            ego_s = s;

            if ego_t > obstacles_info(i).st_overlap(end, 1)
                ego_t = obstacles_info(i).st_overlap(end, 1);
            elseif ego_t < obstacles_info(i).st_overlap(1, 1)
                ego_t = obstacles_info(i).st_overlap(1, 1);
            end

            obs_lb = ... % todo:应考虑本车几何尺寸
                    -planning_para.max_lon_control_err + ...
                    interp1(obstacles_info(i).st_overlap(:, 1), ...
                    obstacles_info(i).st_overlap(:, 2), ego_t);
            obs_ub = ...
                    planning_para.max_lon_control_err + ...
                    interp1(obstacles_info(i).st_overlap(:, 1), ...
                    obstacles_info(i).st_overlap(:, 3), ego_t);

            s_safe_follow = planning_para.s_safe_follow;
            s_safe_overtake = planning_para.s_safe_overtake;

            t = t + step;

            if ego_s > obs_lb && ego_s < obs_ub
                obs_cost = Inf;
                return;
            elseif ego_s < obs_ub + s_safe_overtake && ego_s > obs_ub
                dis_ego2obs = ego_s - (obs_ub + s_safe_overtake);

                scale_factor = 1 / s_safe_overtake;
                dis_ego2obs = dis_ego2obs * scale_factor;

                obs_cost = ...
                        obs_cost + ...
                        planning_para.w_obstacle * dis_ego2obs^2 + ...
                        planning_para.default_safe_dis_penalty * step;
            elseif ego_s < obs_lb && ego_s > obs_lb - s_safe_follow
                dis_ego2obs = ego_s - (obs_lb - s_safe_follow);

                scale_factor = 1 / s_safe_follow;
                dis_ego2obs = dis_ego2obs * scale_factor;

                obs_cost = ...
                        obs_cost + ...
                        planning_para.w_obstacle * dis_ego2obs^2 + ...
                        planning_para.default_safe_dis_penalty * step;
            end
        end
    end
end

function [opt_st_path] = CalculateOptimalSTPath( ...
        closed_list, parent_node, planning_para, st_graph)
    opt_st_path_temp = [];
    
    while parent_node(9) ~= Inf || parent_node(10) ~=  Inf
        opt_st_path_temp = ...
            [opt_st_path_temp; parent_node(5), ...
            parent_node(6), parent_node(7), parent_node(8), parent_node(13)];

        for i = 1  : 1 : length(closed_list)
            if closed_list(i, 1)  == parent_node(9) && ...
               closed_list(i, 2) == parent_node(10)
                parent_node = closed_list(i, :);

                closed_list(i, :) = [];

                break;
            end
        end
    end
    
    opt_st_path_temp = ...
            [opt_st_path_temp;  parent_node(5), parent_node(6), ...
            parent_node(7), parent_node(8), parent_node(13)];

    idx_left = 1;
    idx_right = length(opt_st_path_temp);

    while idx_left < idx_right
        temp = opt_st_path_temp(idx_right, :);
        
        opt_st_path_temp(idx_right, :) = opt_st_path_temp(idx_left, :);
        opt_st_path_temp(idx_left, :) = temp;
        
        idx_left = idx_left + 1;
        idx_right = idx_right - 1;
    end

    opt_st_path = [];
    
    for i = 1 : 1 : length(opt_st_path_temp) - 1
        jerk = ...
                (opt_st_path_temp(i + 1, 4) - opt_st_path_temp(i, 4)) / ...
                st_graph.dt;
        kappa = opt_st_path_temp(i, 5);
        
        opt_st_path = [opt_st_path; opt_st_path_temp(i, 1 : 4), jerk, kappa];

        t0 = opt_st_path_temp(i, 1);
        s0 = opt_st_path_temp(i, 2);
        v0 = opt_st_path_temp(i, 3);
        a0 = opt_st_path_temp(i, 4);
        
        dt = planning_para.opt_path_point_interval;
        num_of_loop = round(st_graph.dt / dt) - 1;

        delta_t = 0;

        for i = 1 : 1 : num_of_loop
            delta_t = delta_t + dt;
    
            t = t0 + delta_t;
            a = a0 + jerk * delta_t;
            v = v0 + a0 * delta_t + 1 / 2 * jerk * delta_t^2;
            s = ...
                    s0 + v0 * delta_t + 1 / 2 * a0 * delta_t^2 + ...
                    1 / 6 * jerk * delta_t^3;
            kappa = ...
                    (opt_st_path_temp(i + 1, 5) - opt_st_path_temp(i, 5)) / ...
                    st_graph.dt * delta_t + opt_st_path_temp(i, 5);
                
            opt_st_path = [opt_st_path; t, s, v, a, jerk, kappa];
        end
    end

    opt_st_path = ...
            [opt_st_path; opt_st_path_temp(end, 1 : 4), ...
            jerk, opt_st_path_temp(end, 5)];
end
