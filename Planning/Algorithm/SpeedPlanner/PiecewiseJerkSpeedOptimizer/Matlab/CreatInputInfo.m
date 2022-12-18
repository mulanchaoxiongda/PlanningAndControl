function [sample_interval, s_bound, ds_bound, dds_bound, ddds_bound, ...
        init_state, end_state, integral_step, heuristic_speed_data, ...
        cruise_speed] = CreatInputInfo()
    load input_info.mat;
    
    sample_interval = planning_para.opt_path_point_interval;
    
    init_state.s = 0;
    init_state.ds = ego_init_state.init_speed;
    init_state.dds = ego_init_state.init_acc;

    end_state.s = 0;
    end_state.ds = 0;
    end_state.dds = 0;
    
    cruise_speed = speed_para.cruise_speed;
    
    integral_step = planning_para.opt_path_point_interval;
    
    num_of_knots = length(opt_st_path(:, 1));

    max_control_err = 2.5;
    dis_margin = 0.1;
    ds = planning_para.max_lon_control_err - max_control_err - dis_margin;
    s_bound.lb = opt_st_path(:, 2) - ds;
    s_bound.ub = opt_st_path(:, 2) + ds;
    
    ds_bound.lb = zeros(num_of_knots, 1);
    for i = 1 : 1 : num_of_knots
        ds_bound.ub(i, 1) = ...
                min(speed_para.speed_limit, ...
                (planning_para.lat_acc_max / opt_st_path(i,6))^0.5);
    end
    
    dds_bound.lb = ones(num_of_knots, 1) * planning_para.acc_min;
    dds_bound.ub = ones(num_of_knots, 1) * planning_para.acc_max;
    
    ddds_bound.lb = ones(num_of_knots - 1, 1) * planning_para.jerk_min;
    ddds_bound.ub = ones(num_of_knots - 1, 1) * planning_para.jerk_max;
    
    heuristic_speed_data.ref_s = opt_st_path(:, 2);
    heuristic_speed_data.cruise_speed = opt_st_path(:, 3);
end
