function [fval] = CostFunc(x)
    num_of_knots = length(x) / 3;
    
    [path_reuseable, sample_interval, speed_data_pre, ...
        reference_speed, l_bound, dl_bound, ddl_bound, dddl_bound, ...
        obstacle_avoid_bound, ego_para, init_state, end_state, ...
        distance_lon, reference_line, control_constraint, road_width, ...
        obstacle_bound, integral_step, lane_width, obs_info] = CreatInputInfo();
    
    delta_s = sample_interval;
    
    w_l = 5;
    w_dl = 20.0;
    w_ddl = 500.0;
    w_dddl = 5000.0;
    
    fval = 0;
    
    for i = 1 : 1 : num_of_knots
        fval = ...
                fval + w_l * x(i)^2 + w_dl * x(i + num_of_knots)^2 + ...
                w_ddl * x(i + num_of_knots * 2)^2;
    end
    
    for i = 1 : 1 : num_of_knots - 1
        fval = ...
                fval + w_dddl * ((x(i + 1 + num_of_knots * 2) - ...
                x(i + num_of_knots * 2)) / delta_s)^2;
    end
end
