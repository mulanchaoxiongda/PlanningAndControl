function [inequ, equ] = NonlinearConstraintFunc(x)
    inequ = [];
    
    num_of_knots = length(x) / 3;
    
    [path_reuseable, sample_interval, speed_data_pre, ...
            reference_speed, l_bound, dl_bound, ddl_bound, dddl_bound, ...
            obstacle_avoid_bound, ego_para, init_state, end_state, ...
            distance_lon, reference_line, control_constraint, road_width, ...
            obstacle_bound, integral_step, lane_width, obs_info] = CreatInputInfo();
    
    d1 = ego_para.rear2front;
    d2 = ego_para.rear2back;
    w  = ego_para.width;
    half_w = w / 2;
        
    for i = 1 : 1 : num_of_knots
        theta = atan(x(i + num_of_knots));
        cos_theta = cos(theta);
        sin_theta = sin(theta);
        
        l = x(i);
        dl = x(i + num_of_knots);
        
        lb_head = obstacle_avoid_bound.lb_head(i);
        ub_head = obstacle_avoid_bound.ub_head(i);
        lb_back = obstacle_avoid_bound.lb_back(i);
        ub_back = obstacle_avoid_bound.ub_back(i);
        
        inequ = [inequ;
                 -l - d1 * sin_theta - half_w * cos_theta + lb_head;
                 l + d1 * sin_theta + half_w * cos_theta - ub_head;
                 -l - d1 * sin_theta + half_w * cos_theta + lb_head;
                 l + d1 * sin_theta - half_w * cos_theta - ub_head;
                 -l + d2 * sin_theta - half_w * cos_theta + lb_back;
                 l - d2 * sin_theta + half_w * cos_theta - ub_back;
                 -l + d2 * sin_theta + half_w * cos_theta + lb_back;
                 l - d2 * sin_theta - half_w * cos_theta - ub_back];
    end

    for i = 1 : 1 : num_of_knots
        kappa_ub = ddl_bound.ub(i);
        kappa_lb = ddl_bound.lb(i);

%         inequ = [inequ;
%                  x(i + num_of_knots * 2) / (1 + x(i + num_of_knots)^2)^(3 / 2) - kappa_ub;
%                  -x(i + num_of_knots * 2) / (1 + x(i + num_of_knots)^2)^(3 / 2) + kappa_lb];

        l = x(i);
        dl = x(i + num_of_knots);
        ddl = x(i + num_of_knots * 2);

        kappa_ref = reference_line.kappa(i);
        dkappa_ref = reference_line.dkappa_ds(i);

        delta_theta = atan(dl / (1 - kappa_ref * l));

        kappa_global = ...
                (((ddl + (dkappa_ref * l + kappa_ref * dl) * ...
                tan(delta_theta)) * cos(delta_theta)^2) / ...
                (1 - kappa_ref * l) + kappa_ref) * ...
                cos(delta_theta) / (1 - kappa_ref * l);
        
        kappa_ub = ddl_bound.ub(i) + kappa_ref;
        kappa_lb = ddl_bound.lb(i) + kappa_ref;
        
        inequ = [inequ;
                 kappa_global - kappa_ub;
                 -kappa_global + kappa_lb];
    end
    
    delta_s = sample_interval;
    
    for i = 1 : 1 : num_of_knots - 1
        l = x(i);
        dl = x(i + num_of_knots);
        ddl = x(i + num_of_knots * 2);
        dddl = ...
                (x(i + 1 + num_of_knots * 2) - ...
                x(i + num_of_knots * 2)) / delta_s;
            
        molecule = ...
                dddl * (1 + dl^2)^(3/2) - ...
                ddl * 3/2 * (1 + dl^2)^0.5 * (2 * dl * ddl);
        denominator = (1 + dl^2)^3;
        
        dkappa_ub = dddl_bound.ub(i);
        dkappa_lb = dddl_bound.lb(i);
        
        inequ = [inequ;
                 molecule / denominator - dkappa_ub;
                 -molecule / denominator + dkappa_lb];
             
        kappa_ref = reference_line.kappa(i);
        dkappa_ref = reference_line.dkappa_ds(i);
        ddkappa_ref = ...
                (reference_line.dkappa_ds(i + 1) - reference_line.dkappa_ds(i)) / ...
                delta_s;

        delta_theta = atan(dl / (1 - kappa_ref * l));

        kappa_global = ...
                (((ddl + (dkappa_ref * l + kappa_ref * dl) * ...
                tan(delta_theta)) * cos(delta_theta)^2) / ...
                (1 - kappa_ref * l) + kappa_ref) * ...
                cos(delta_theta) / (1 - kappa_ref * l);
        
        dkappa_global = ...
                (((dddl + (ddkappa_ref * l + 2 * dkappa_ref * dl + kappa_ref * ddl) * ...
                tan(delta_theta) + (dkappa_ref * l + kappa_ref * dl) * sec(delta_theta)^2 * ...
                (kappa_global * (1 - kappa_ref * l) / cos(delta_theta) - kappa_ref)) * ...
                cos(delta_theta)^2 + (ddl + (dkappa_ref * l + kappa_ref * dl) * ...
                tan(delta_theta)) * 2 * cos(delta_theta) * (-sin(delta_theta)) * ...
                (kappa_global * (1 - kappa_ref * l) / cos(delta_theta) - kappa_ref)) * ...
                (1 - kappa_ref * l) - (ddl + (dkappa_ref * l + kappa_ref * dl) * ...
                tan(delta_theta)) * cos(delta_theta)^2 * (1 - dkappa_ref * l - kappa_ref * dl) / ...
                (1 - kappa_ref * l)^2 + dkappa_ref) * cos(delta_theta) / (1 - kappa_ref * l) + ...
                ((ddl + (dkappa_ref * l + kappa_ref * dl) * tan(delta_theta)) * cos(delta_theta)^2 / ...
                (1 - kappa_ref * l) + kappa_ref) * (-sin(delta_theta)) * (kappa_global * ...
                (1 - kappa_ref * l) / cos(delta_theta) - kappa_ref) * (1 - kappa_ref * l) - ...
                cos(delta_theta) * (1 - dkappa_ref * l - kappa_ref * dl) / (1 - kappa_ref * l)^2;
        
        dkappa_ub = dddl_bound.ub(i) + dkappa_ref;
        dkappa_lb = dddl_bound.lb(i) + dkappa_ref;
        
%         inequ = [inequ;
%                  dkappa_global - dkappa_ub;
%                  -dkappa_global + dkappa_lb];
    end
    
    equ = [];
end
