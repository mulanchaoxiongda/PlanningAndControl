function [path_data, path_solver_status, opt_solution] = ...
        PiecewiseJerkPathOptimizer(path_reuseable, sample_interval, l_bound, ...
        dl_bound, ddl_bound, dddl_bound, obstacle_avoid_bound, ego_para, ...
        init_state, end_state, integral_step)
	%% path_data_pqre reuseable
    if path_reuseable
        path_solver_status = 2;
        return;
    end
    
    num_of_knots = length(l_bound.lb);
    
    %% cost function: CostFunc()

    %% nonlinear constraint: NonlinearConstraintFunc()

    %% equality constraint matix_A_equal(3 + 2 * (num_of_knots - 1), 3 * num_of_knots) && vector_b_equal(3 + 2 * (num_of_knots - 1), 1)
    matrix_A_init_state = zeros(3, num_of_knots * 3);
    matrix_A_init_state(1, 1) = 1;
    matrix_A_init_state(2, num_of_knots + 1) = 1;
    matrix_A_init_state(3, num_of_knots * 2 + 1) = 0;

    vector_b_init_state = [init_state.l; init_state.dl; init_state.ddl*0];

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
    
    matrix_Aequ = [matrix_A_init_state; matrix_A_motion_model; matrix_A_Motion_model];    
    vector_bequ = [vector_b_init_state; vector_b_motion_model; vector_b_Motion_model];

    %% inequality constraint matrix_A_inequ((num_of_knots - 1) * 2 + num_of_knots * 4, 3 * num_of_knots) && vector_b_inequ((num_of_knots - 1) * 2 + num_of_knots * 4, 1)
    matrix_A = [];
    vector_b = [];
    
    %% variable constraint
    l_ub =  ones(num_of_knots, 1) * 10000000;
    l_lb = -ones(num_of_knots, 1) * 10000000;
    ddl_ub =  ones(num_of_knots, 1) * 10000000;
    ddl_lb = -ones(num_of_knots, 1) * 10000000;
    vector_ub = [l_ub; dl_bound.ub; ddl_ub];
    vector_lb = [l_lb; dl_bound.lb; ddl_lb];
    
    %% x0
    x0 = zeros(1, 3 * num_of_knots);
    
    %% qp solver
    options = optimoptions('fmincon', 'Algorithm', 'interior-point', 'TolFun', 10^(-6), 'ObjectiveLimit', 10^(-6), 'Display', 'iter-detailed', 'MaxFunEvals', 10000000, 'MaxIter', 200, 'PlotFcns', 'optimplotx'); % 内点法(interior point)；序列二次规划法(SQP)；有效集法(active set)；信頼域法(trust region reflective)
    [opt_solution, fval, exitflag, output] = fmincon('CostFunc', x0, matrix_A, vector_b, matrix_Aequ, vector_bequ, vector_lb, vector_ub, 'NonlinearConstraintFunc', options);

    path_solver_status = 1;

    % calculate path
    [path_data] = ...
            CalculatePathFunc(sample_interval, num_of_knots, ...
            opt_solution, init_state, integral_step);
end

%% calculate path function
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
    0
end
