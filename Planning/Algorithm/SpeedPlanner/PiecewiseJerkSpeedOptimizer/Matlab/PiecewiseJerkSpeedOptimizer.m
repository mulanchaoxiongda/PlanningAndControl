function [speed_data, opt_solution] = ...
        PiecewiseJerkSpeedOptimizer(sample_interval, s_bound, ...
        ds_bound, dds_bound, ddds_bound, init_state, end_state, ...
        integral_step, heuristic_speed_data, cruise_speed)
    % constraint switch
    end_state_hard_constraint_switch = false;
    
    % cost weight coeff
    w_s = 1;
    w_ds = 100;
    w_dds = 150;
    w_ddds = 500;
    
    num_of_knots = length(s_bound.lb);
    
    % calculate H matrix(3 * num_of_knots, 3 * num_of_knots)
    matrix_H_s = eye(num_of_knots) * w_s;
    
    matrix_H_ds = eye(num_of_knots) * w_ds;
    
    matrix_H_dds = ...
            eye(num_of_knots) * (w_dds + 2 * w_ddds / sample_interval^2);
	matrix_H_dds(1, 1) = matrix_H_dds(1, 1) - w_ddds / sample_interval^2;
    matrix_H_dds(num_of_knots, num_of_knots) = ...
            matrix_H_dds(num_of_knots, num_of_knots) - ...
            w_ddds / sample_interval^2;
    for row = 2 : 1 : num_of_knots
        matrix_H_dds(row, row - 1) = -2 * w_ddds / sample_interval^2;
    end
    
    matrix_H = blkdiag(matrix_H_s, matrix_H_ds, matrix_H_dds);
    
    % calculate f vector(3 * num_of_knots, 1)
    for i = 1 : 1 : num_of_knots
        vector_f_s(i, 1) = -2 * w_s * heuristic_speed_data.ref_s(i);
    end
    
    for i = 1 : 1 : num_of_knots
        vector_f_ds(i, 1) = -2 * w_ds * heuristic_speed_data.cruise_speed(i);
    end
    
    vector_f_dds = zeros(num_of_knots, 1);
    
    vector_f = [vector_f_s; vector_f_ds; vector_f_dds];
    
    % equality constraint matrix_A_equal(3 * 2 + 2 * (num_of_knots - 1), 3 * num_of_knots) &&
    % vector_b_equal(3 * 2 + 2 * (num_of_knots - 1), 1)
    matrix_A_init_state = zeros(3, num_of_knots * 3);
    matrix_A_init_state(1, 1) = 1;
    matrix_A_init_state(2, num_of_knots + 1) = 1;
    matrix_A_init_state(3, num_of_knots * 2 +1) = 1;
    
    vector_b_init_state = [init_state.s; init_state.ds; init_state.dds];
    
    matrix_A_end_state(1, num_of_knots) = 1;
    matrix_A_end_state(2, num_of_knots * 2) = 1;
    matrix_A_end_state(3, num_of_knots * 3) = 1;
    
    vector_b_end_state = [end_state.s; end_state.ds; end_state.dds];
    
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
            
    % inequality constraint matrix_A_inequ((num_of_knots 1) * 2, 3 * 
    % num_of_knots) && vector_b_inequ((num_of_knots - 1) * 2, 1);
    matrix_Ainequ_ddds = zeros((num_of_knots - 1) * 2, 3 * num_of_knots);
    for row = 1 : num_of_knots - 1
        col = row + num_of_knots * 2;
        matrix_Ainequ_ddds(row, col : col + 1) = ...
                [-1 / sample_interval, 1 / sample_interval];
    end
    for row = num_of_knots : (num_of_knots - 1) * 2
        col = row - (num_of_knots - 1) + num_of_knots * 2;
        matrix_Ainequ_ddds(row, col : col + 1) = ...
                [1 / sample_interval, -1 / sample_interval];
    end
    
    vector_Binequ_ddds = zeros((num_of_knots - 1) * 2, 1);
    vector_Binequ_ddds(1 : (num_of_knots - 1) * 2, 1) = ...
            [ddds_bound.ub; -ddds_bound.lb];
    
    matrix_A_inequ = matrix_Ainequ_ddds;
    vector_b_inequ = vector_Binequ_ddds;
    
    % variable constraint
    vector_ub = [s_bound.ub; ds_bound.ub; dds_bound.ub];
    vector_lb = [s_bound.lb; ds_bound.lb; dds_bound.lb];

    % qp solver
    options = optimoptions('quadprog', 'MaxIterations', 2000, 'TolFun', 1e-6);
    opt_solution = ...
            quadprog(matrix_H + matrix_H', vector_f, ...
            matrix_A_inequ, vector_b_inequ, matrix_A_equal, ...
            vector_b_equal, vector_lb, vector_ub, [], options);
    
    % calculate speed_data
    [speed_data] = ...
            CalculateSpeedDataFunc(sample_interval, num_of_knots, ...
            opt_solution, init_state, integral_step);
end

function [speed_data] = CalculateSpeedDataFunc( ...
        sample_interval, num_of_knots, opt_solution, ...
        init_state, integral_step)
    opt.t = [0 : 1 : num_of_knots - 1] * sample_interval;
    opt.s = opt_solution(1 : num_of_knots);
    opt.ds = opt_solution(num_of_knots + 1 : num_of_knots * 2);
    opt.dds = opt_solution(num_of_knots * 2 + 1 : num_of_knots * 3);
    opt.ddds = zeros(num_of_knots - 1, 1);
    for i = 1 : 1 : num_of_knots - 1
        opt.ddds(i) = (opt.dds(i + 1) - opt.dds(i)) / sample_interval;
    end

    speed_data.t = [];
    speed_data.s = [];
    speed_data.ds = [];
    speed_data.dds = [];
    speed_data.init_state = init_state;
    speed_data.opt_result = opt;
    
    step = integral_step;
    point_num_fragment = ceil(sample_interval / step);
    step = sample_interval / point_num_fragment;
    
    for i = 1 : 1 : num_of_knots - 1
        idx = 1 + (i - 1) * point_num_fragment;
        
        speed_data.t(idx) = opt.t(i);
        speed_data.s(idx) = opt.s(i);
        speed_data.ds(idx) = opt.ds(i);
        speed_data.dds(idx) = opt.dds(i);
        speed_data.ddds(idx) = opt.ddds(i);
        
        for j = 1 : 1 : point_num_fragment - 1
            idx = j + 1 + (i - 1) * point_num_fragment;
            
            speed_data.t(idx) = speed_data.t(idx - 1) + step;
            speed_data.ddds(idx) = opt.ddds(i);
            speed_data.dds(idx) = ...
                    speed_data.dds(idx - 1) + speed_data.ddds(idx) * step;
            speed_data.ds(idx) = ...
                    speed_data.ds(idx - 1) + speed_data.dds(idx - 1) * step + ...
                    0.5 * speed_data.ddds(idx - 1) * step^2;
            speed_data.s(idx) = ...
                    speed_data.s(idx - 1) + speed_data.ds(idx - 1) * step + ...
                    1 / 2 * speed_data.dds(idx - 1) * step^2 + ...
                    1 / 6 * speed_data.ddds(idx) * step^3;
        end
    end
    
    idx = 1 + (num_of_knots - 1) * point_num_fragment;
    
    speed_data.t(idx) = opt.t(num_of_knots);
    speed_data.s(idx) = opt.s(num_of_knots);
    speed_data.ds(idx) = opt.ds(num_of_knots);
    speed_data.dds(idx) = opt.dds(num_of_knots);
    speed_data.ddds(idx) = opt.ddds(num_of_knots - 1);
    
    speed_data.step = step;
end
