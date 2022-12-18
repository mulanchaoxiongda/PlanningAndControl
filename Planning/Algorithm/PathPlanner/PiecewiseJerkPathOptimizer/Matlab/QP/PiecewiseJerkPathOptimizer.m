function [path_data, path_solver_status, opt_solution] = ...
        PiecewiseJerkPathOptimizer(path_reuseable, sample_interval, l_bound, ...
        dl_bound, ddl_bound, dddl_bound, obstacle_avoid_bound, ego_para, ...
        init_state, end_state, integral_step)
	% path_data_pre reuseable
    if path_reuseable
        path_solver_status = 2;
        return;
    end
    
    % constraint switch
    obs_avoid_soft_constraint_switch = false;
    end_state_soft_constraint_switch = false;
    end_state_hard_constraint_switch = false;
    
    % cost weight coeff
    w_l = 0.2;
    w_dl = 40.0;
    w_ddl = 1000.0;
    w_dddl = 100000.0;
    
    w_obs = 1.0 * obs_avoid_soft_constraint_switch;
    w_end_l = 1.0 * end_state_soft_constraint_switch;
    w_end_dl = 1.0 * end_state_soft_constraint_switch;
    w_end_ddl = 1.0 * end_state_soft_constraint_switch;
    
    num_of_knots = length(l_bound.lb);
    
    % calculate H matrix(3 * num_of_knots, 3 * num_of_knots)
    matrix_H_l = eye(num_of_knots) * (w_l + w_obs);
    matrix_H_l(num_of_knots, num_of_knots) = ...
            matrix_H_l(num_of_knots, num_of_knots) + w_end_l;
        
    matrix_H_dl = eye(num_of_knots) * w_dl;
    matrix_H_dl(num_of_knots, num_of_knots)= ...
            matrix_H_dl(num_of_knots, num_of_knots) + w_end_dl;
        
    matrix_H_ddl = ...
            eye(num_of_knots) * (w_ddl + 2 * w_dddl / sample_interval^2);
    matrix_H_ddl(1, 1) = matrix_H_ddl(1, 1) - w_dddl / sample_interval^2;
    matrix_H_ddl(num_of_knots, num_of_knots) = ...
            matrix_H_ddl(num_of_knots, num_of_knots) - ...
            w_dddl / sample_interval^2;
    for row = 2 : 1 : num_of_knots
        col = row - 1;
        matrix_H_ddl(row, col) = -2 * w_dddl / sample_interval^2;
    end
    matrix_H_ddl(num_of_knots, num_of_knots) = ...
            matrix_H_ddl(num_of_knots, num_of_knots) + w_end_ddl;
        
    matrix_H = blkdiag(matrix_H_l, matrix_H_dl, matrix_H_ddl);
    
    % calculate f matrix(3 * num_of_knots, 1)
    for i = 1 : 1 : num_of_knots
        vector_f_l(i, 1) = -2 * w_obs * (l_bound.lb(i) + l_bound.ub(i)) / 2;
    end
    vecotr_f_l(num_of_knots, 1) = ...
            vector_f_l(num_of_knots, 1) - 2 * w_end_l * end_state.l;

    vector_f_dl = zeros(num_of_knots, 1);
    vector_f_dl(num_of_knots, 1) = -2 * w_end_dl * end_state.dl;
    
    vector_f_ddl = zeros(num_of_knots, 1);
    vector_f_ddl(num_of_knots, 1) = -2 * w_end_ddl * end_state.ddl;
    
    vector_f = [vector_f_l; vector_f_dl; vector_f_ddl];
    
    % equality constraint matix_A_equal(6 + 2 * (num_of_knots - 1), 3 *
    % num_of_knots) && vector_b_equal(6 + 2 * (num_of_knots - 1), 1)
    matrix_A_init_state = zeros(3, num_of_knots * 3);
    matrix_A_init_state(1, 1) = 1;
    matrix_A_init_state(2, num_of_knots + 1) = 1;
    matrix_A_init_state(3, num_of_knots * 2 + 1) = 1;
    
    vector_b_init_state = [init_state.l; init_state.dl; init_state.ddl];
    
    matrix_A_end_state = zeros(3, num_of_knots * 3);
    matrix_A_end_state(1, num_of_knots) = 1;
    matrix_A_end_state(2, num_of_knots * 2) = 1;
    matrix_A_end_state(3, num_of_knots * 3) = 1;
    
    vector_b_end_state = [end_state.l; end_state.dl; end_state.ddl];
    
    matrix_A_end_state = ...
            matrix_A_end_state * end_state_hard_constraint_switch;
    vector_b_end_state = ...
            vector_b_end_state * end_state_hard_constraint_switch;
        
    matrix_A_motion_model = zeros(num_of_knots - 1, num_of_knots * 3);
    
    for row = 1 : 1 : num_of_knots - 1
        col = row + num_of_knots;
        matrix_A_motion_model(row, col : col + 1) = [-1, 1];
        
        col = col + num_of_knots;
        matrix_A_motion_model(row, col : col + 1) = ...
                [-sample_interval / 2, -sample_interval / 2];
    end
    
    vector_b_motion_model = zeros(num_of_knots - 1, 1);
    
    matrix_A_Motion_model = zeros(num_of_knots - 1, num_of_knots * 3);
        
    for row = 1 : 1 : num_of_knots - 1
        col = row;
        matrix_A_Motion_model(row, col : col + 1) = [-1, 1];
        
        col = col + num_of_knots;
        matrix_A_Motion_model(row, col) = -sample_interval;
            
        col = col + num_of_knots;
        matrix_A_Motion_model(row, col : col + 1) = ...
                [-sample_interval^2 / 3, -sample_interval^2 / 6];
    end
    
    vector_b_Motion_model = zeros(num_of_knots - 1, 1);
    
    matrix_A_equal = ...
            [matrix_A_init_state; matrix_A_end_state; ...
             matrix_A_motion_model; matrix_A_Motion_model];    
     vector_b_equal = ...
            [vector_b_init_state; vector_b_end_state; ...
             vector_b_motion_model; vector_b_Motion_model];
         
    % inequality constraint matrix_A_inequ((num_of_knots - 1) * 2 + num_of_knots * 4, 3 *
    % num_of_knots) && vector_b_inequ((num_of_knots - 1) * 2 + num_of_knots * 4, 1)
    matrix_A_inequ_dddl = zeros((num_of_knots - 1) * 2, 3 * num_of_knots);
    for row = 1 : num_of_knots - 1
        col = row + num_of_knots * 2;
        matrix_A_inequ_dddl(row, col : col + 1) = ...
                [-1 / sample_interval, 1 / sample_interval];
    end
    for row = num_of_knots : (num_of_knots - 1) * 2
        col = row - (num_of_knots - 1) + num_of_knots * 2;
        matrix_A_inequ_dddl(row, col : col + 1) = ...
                [1 / sample_interval, -1 / sample_interval];
    end
    
    vector_b_inequ_dddl = zeros((num_of_knots - 1) * 2, 1);
    vector_b_inequ_dddl(1 : (num_of_knots - 1) * 2, 1) = ...
            [dddl_bound.ub; -dddl_bound.lb];
        
    matrix_A_inequ_obs = zeros(num_of_knots * 4, 3 * num_of_knots);
    for row = 1 : 1 :num_of_knots
        col = row;
        matrix_A_inequ_obs(row, col) = 1;
        matrix_A_inequ_obs(row, col + num_of_knots) = ego_para.rear2front;
    end
    for row = num_of_knots + 1 : 1 :num_of_knots * 2
        col = row - num_of_knots;
        matrix_A_inequ_obs(row, col) = -1;
        matrix_A_inequ_obs(row, col + num_of_knots) = -ego_para.rear2front;
    end
    for row = num_of_knots * 2 + 1 : 1 : num_of_knots * 3
        col = row - num_of_knots * 2;
        matrix_A_inequ_obs(row, col) = 1;
        matrix_A_inequ_obs(row, col + num_of_knots) = -ego_para.rear2back;
    end
    for row = num_of_knots * 3 + 1 : 1 : num_of_knots * 4
        col = row - num_of_knots * 3;
        matrix_A_inequ_obs(row, col) = -1;
        matrix_A_inequ_obs(row, col + num_of_knots) = ego_para.rear2back;
    end
    
    vector_b_inequ_obs = zeros(num_of_knots * 4, 1);
    vector_b_inequ_obs(1 : num_of_knots * 2, 1) = ...
            [obstacle_avoid_bound.ub(1 : num_of_knots, 1); ...
             obstacle_avoid_bound.lb(1 : num_of_knots, 1) * (-1)];
    vector_b_inequ_obs(num_of_knots * 2 + 1 : num_of_knots * 4, 1) = ...
            [obstacle_avoid_bound.ub(num_of_knots + 1 : num_of_knots * 2, 1); ...
             -obstacle_avoid_bound.lb(num_of_knots + 1 : num_of_knots * 2, 1)];

    matrix_A_inequ = [matrix_A_inequ_dddl; matrix_A_inequ_obs];
    vector_b_inequ = [vector_b_inequ_dddl; vector_b_inequ_obs];
    
    % variable constraint
    vector_var_ub = [l_bound.ub; dl_bound.ub; ddl_bound.ub];
    vector_var_lb = [l_bound.lb; dl_bound.lb; ddl_bound.lb];
    
    %qp solver
    options = optimoptions('quadprog', 'MaxIterations', 2000, 'TolFun', 1e-6);
    opt_solution = ...
            quadprog(matrix_H + matrix_H', vector_f, ...
            matrix_A_inequ, vector_b_inequ, matrix_A_equal, ...
            vector_b_equal, vector_var_lb, vector_var_ub, [], options);
        
    path_solver_status = 1;
    
    % calculate path
    [path_data] = ...
            CalculatePathFunc(sample_interval, num_of_knots, ...
            opt_solution, init_state, integral_step);
end

function [path_data] = CalculatePathFunc( ...
        sample_interval, num_of_knots, opt_solution, init_state, integral_step)
    opt.s = [0 : 1 : num_of_knots - 1] * sample_interval;
    opt.l = opt_solution(1 : num_of_knots);
    opt.dl = opt_solution(num_of_knots + 1 : num_of_knots * 2);
    opt.ddl = opt_solution(num_of_knots * 2 + 1 : num_of_knots * 3);
    opt.dddl = zeros(num_of_knots - 1, 1);
    for i = 1 : 1 : num_of_knots - 1
        opt.dddl(i) = (opt.ddl(i + 1) - opt.ddl(i)) / sample_interval;
    end
    
    path_data.s = [];
    path_data.l = [];
    path_data.dl = [];
    path_data.ddl = [];
    path_data.init_state = init_state;
    path_data.opt_solution = opt;

    step = integral_step;
    point_num_fragment = ceil(sample_interval / step);
    step = sample_interval / point_num_fragment;
    
    for i = 1 : 1 : num_of_knots - 1
        idx = 1 + (i - 1) * point_num_fragment;
        
        path_data.s(idx) = opt.s(i);
        path_data.l(idx) = opt.l(i);
        path_data.dl(idx) = opt.dl(i);
        path_data.ddl(idx) = opt.ddl(i);
        path_data.dddl(idx) = opt.dddl(i);
    
        for j = 1 : 1 : point_num_fragment - 1
            idx = j + 1 + (i - 1) * point_num_fragment;
            
            path_data.s(idx) = path_data.s(idx - 1) + step;
            path_data.dddl(idx) = opt.dddl(i);
            path_data.ddl(idx) = ...
                    path_data.ddl(idx - 1) + path_data.dddl(idx - 1) * step; % correct
            path_data.dl(idx) = ...
                    path_data.dl(idx - 1) + path_data.ddl(idx - 1) * step + ...
                    0.5 * path_data.dddl(idx - 1) * step^2;
            path_data.l(idx) = ...
                    path_data.l(idx - 1) + path_data.dl(idx - 1) * step + ...
                    0.5 * path_data.ddl(idx - 1) * step^2 + ...
                    1 / 6 * path_data.dddl(idx - 1) * step^3; % correct
        end
    end
    
    idx = 1 + (num_of_knots - 1) * point_num_fragment;
    
    path_data.s(idx) = opt.s(num_of_knots);
    path_data.l(idx) = opt.l(num_of_knots);
    path_data.dl(idx) = opt.dl(num_of_knots);
    path_data.ddl(idx) = opt.ddl(num_of_knots);
    path_data.dddl(idx) = opt.dddl(num_of_knots - 1);
    
    path_data.step = step;
    
end
