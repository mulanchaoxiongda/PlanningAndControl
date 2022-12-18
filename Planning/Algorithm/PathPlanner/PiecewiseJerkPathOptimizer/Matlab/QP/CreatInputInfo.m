function [path_reuseable, sample_interval, speed_data_pre, ...
        reference_speed, l_bound, dl_bound, ddl_bound, dddl_bound, ...
        obstacle_avoid_bound, ego_para, init_state, end_state, ...
        distance_lon, reference_line, control_constraint, road_width, ...
        obstacle_bound, integral_step, lane_width, obs_info] = CreatInputInfo()
    % vehicle simulation result:reference_speed = 20 / 3.6, obs_speed = 10 / 3.6, total = 12, path_length = 67;
    % truck   simulation result:reference_speed = 20 / 3.6, obs_speed = 10 / 3.6, total = 22, path_length =123;
    reference_speed = 20 / 3.6;
    total_time = 10;
    time_sample_interval = 0.5;
    
    obs_speed = reference_speed - 10 / 3.6;
    if obs_speed < 0.0001 / 3.6
        disp( '[warning]obstacle speed is negative!');   
    end

    time_collision = total_time / 2;
    turning_radius = 1000000.0;
     
    integral_step = 0.25 * reference_speed;
    
    has_noise_perception = false;
    is_static_obs = false;
    
    distance_lon = total_time * reference_speed;
    sample_interval =reference_speed * time_sample_interval;
    num_of_knots = round(distance_lon / sample_interval + 1);
    
    ego_para.wheelbase      = 2.84;
    ego_para.width          = 2.06;
    ego_para.length         = 4.70;
    ego_para.rear2front     = 3.70;
    ego_para.rear2back      = 1.00;
    ego_para.steer_ratio    = 16.0;
    ego_para.steer_max      = 8.20;
    ego_para.steer_rate_max = 6.98;
    
    road_width = 3.5 * 3; % 3.5m * 3 lanes
    heading_max = 45.0 / 57.3;
    lane_width = road_width / 3;
    
    path_reuseable     = false;
    speed_data_pre     = zeros(num_of_knots, 1);
    
    [reference_line] = ...
            GenerateReferenceLine(turning_radius, sample_interval, ...
            num_of_knots, reference_speed);
        
    [control_constraint] = ...
            CalControlConstraint(reference_speed, ego_para);
        
    l_bound.lb = -ones(num_of_knots, 1) * road_width / 2;
    l_bound.ub =  ones(num_of_knots, 1) * road_width / 2;
    
    dl_bound.lb = -ones(num_of_knots, 1) * tan(heading_max);
    dl_bound.ub =  ones(num_of_knots, 1) * tan(heading_max);
    
    ddl_bound.lb = ...
            -ones(num_of_knots, 1) * tan(control_constraint.steer_max) / ...
            ego_para.wheelbase -reference_line.kappa;
    ddl_bound.ub = ...
            ones(num_of_knots, 1) * tan(control_constraint.steer_max) / ...
            ego_para.wheelbase -reference_line.kappa;
        
    dddl_bound.lb = ...
            -ones(num_of_knots - 1, 1) * control_constraint.steer_rate_max / ...  
            ego_para.wheelbase / reference_speed - ...
            reference_line.dkappa_ds(1 : num_of_knots - 1);
    dddl_bound.ub = ...
            ones(num_of_knots - 1, 1) * control_constraint.steer_rate_max / ... 
            ego_para.wheelbase / reference_speed - ...
            reference_line.dkappa_ds(1 : num_of_knots - 1);
        
    [obstacle_avoid_bound, obstacle_bound, obs_info] = ...
            GenerateObstacles(num_of_knots, ego_para, reference_speed, road_width, ...
            sample_interval, time_sample_interval, time_collision, lane_width, ...
            has_noise_perception, obs_speed, is_static_obs);
        
    init_state.l   = 0.2;
    init_state.dl  = 0;
    init_state.ddl = 0;
        
    end_state.l   = 0.0;
    end_state.dl  = 0.0;
    end_state.ddl = 0.0;
end

function [reference_line] = GenerateReferenceLine( ...
        turning_radius, sample_interval, num_of_knots, reference_speed)
    for idx = 1 : 1 : num_of_knots
        center_corner = (idx - 1) * sample_interval / turning_radius; % turn left
         
        reference_line.x(idx, 1) = cos(center_corner) * turning_radius;
        reference_line.y(idx, 1) = sin(center_corner) * turning_radius;
        reference_line.theta(idx, 1) = center_corner + pi / 2;
        reference_line.kappa(idx, 1) = 1 / turning_radius;
        reference_line.dkappa_dt(idx, 1) = 0;
    end
    
    reference_line.dkappa_ds = reference_line.dkappa_dt * reference_speed;
    
    reference_line.reference_speed = reference_speed;
    reference_line.sample_interval = sample_interval;
    reference_line.turning_radius  = turning_radius;
    reference_line.num_of_knots    = num_of_knots;
end

function [control_constraint] = CalControlConstraint( ...
       reference_speed, ego_para)
   switch reference_speed
       case 5 / 3.6
           steer_max = 433 / ego_para.steer_ratio / 57.3;
       case 20 / 3.6 
           steer_max = 164 / ego_para.steer_ratio / 57.3;
       case 40 / 3.6
           steer_max = 62 / ego_para.steer_ratio / 57.3;
       case 80 / 3.6
           steer_max = 15 / ego_para.steer_ratio / 57.3;
       case 120 / 3.6  
           steer_max = 7 / ego_para.steer_ratio / 57.3;
       otherwise
           disp('[error]reference speed is out of range!');
   end
   
   control_constraint.steer_max = steer_max;
   control_constraint.steer_rate_max = ...
           ego_para.steer_rate_max / ego_para.steer_ratio / 2.0;
       
   if control_constraint.steer_rate_max > control_constraint.steer_max
       control_constraint.steer_rate_max = control_constraint.steer_max;
   end
   
   control_constraint.kappa_max = tan(steer_max) / ego_para.wheelbase;
   control_constraint.kappa_rate_max = ...
           control_constraint.steer_rate_max / ...
           ego_para.wheelbase / reference_speed;
end

function [obstacle_avoid_bound, obstacle_bound, obs_info] = GenerateObstacles( ...
            num_of_knots, ego_para, reference_speed, road_width, sample_interval, ...
            time_sample_interval, time_collision, lane_width, ...
            has_noise_perception, obs_speed, is_static_obs)
   obs_info.s = reference_speed * time_collision;
   obs_info.l = 0;
   
   obs_info.width = ego_para.width;
   
   if is_static_obs == true
       obs_info.length = 5;
   else
       obs_info.length = ...
               (ego_para.length + ego_para.length) / ...
               (reference_speed - obs_speed) * obs_speed + ego_para.length;
   end
   
   safe_distance = 0.5;
   
   start_pos = ...
           reference_speed * time_collision - obs_info.length / 2 - ...
           sample_interval;
   end_pos   = ...
           reference_speed * time_collision + obs_info.length / 2 + ...
           sample_interval;
       
   time_margin = (time_sample_interval + 0.0001);
   
   for idx = 1 : 1 : num_of_knots
       s = (idx - 1) * sample_interval;
       
       if s < start_pos || s > end_pos
           obstacle_bound.lb_head(idx, 1) = -road_width / 2;
           obstacle_bound.ub_head(idx, 1) =  road_width / 2;
           
           obstacle_bound.lb_back(idx, 1) = -road_width / 2;
           obstacle_bound.ub_back(idx, 1) =  road_width / 2;
       else
           obstacle_bound.lb_head(idx, 1) = -road_width / 2;
           obstacle_bound.ub_head(idx, 1) = -obs_info.width / 2;
           
           obstacle_bound.lb_back(idx, 1) = -road_width / 2;
           obstacle_bound.ub_back(idx, 1) = -road_width / 2;
       end
       
       if s < start_pos - ego_para.rear2front || ...
          s > end_pos + ego_para.rear2back % if s < start_pos - ego_para.rear2front * 0.1 || s> end_pos + ego_para.rear2back
           obstacle_avoid_bound.lb_head(idx, 1) = ...
                   - road_width / 2 + obs_info.width / 2; % Todo : ignore constraints between front steer && rear steer
           obstacle_avoid_bound.ub_head(idx, 1) = ...
                   road_width / 2 - obs_info.width / 2;
              
           obstacle_avoid_bound.lb_back(idx, 1) = ...  
                   - road_width / 2 + obs_info.width / 2;
           obstacle_avoid_bound.ub_back(idx, 1) = ...    
                   road_width / 2 - obs_info.width / 2;
       else
           obstacle_avoid_bound.lb_head(idx, 1) = ...
                   - road_width / 2 + safe_distance + obs_info.width / 2;
           obstacle_avoid_bound.ub_head(idx, 1) = ...
                   - obs_info.width / 2 - safe_distance - obs_info.width / 2;
               
           obstacle_avoid_bound.lb_back(idx, 1) = ... 
                   - road_width / 2 + safe_distance + obs_info.width / 2;
           obstacle_avoid_bound.ub_back(idx, 1) = ...
                   - obs_info.width / 2 - safe_distance - obs_info.width / 2;
       end
   end
   
   for i = 1 : 1 : length(obstacle_avoid_bound.lb_back)
       noise_lb = normrnd(0, 0.5) * has_noise_perception;
       noise_ub = normrnd(0, 0.5) * has_noise_perception;
       
       obstacle_avoid_bound.lb_head(i) = ...
                obstacle_avoid_bound.lb_head(i) + noise_lb;
       obstacle_avoid_bound.lb_back(i) = ...
                obstacle_avoid_bound.lb_back(i) + noise_lb;
            
       obstacle_bound.lb_head(i) = obstacle_bound.lb_head(i) + noise_lb;
       obstacle_bound.lb_back(i) = obstacle_bound.lb_back(i) + noise_lb;
       
       obstacle_avoid_bound.ub_head(i) = ... 
                obstacle_avoid_bound.ub_head(i) + noise_ub;
       obstacle_avoid_bound.ub_back(i) = ...    
                obstacle_avoid_bound.ub_back(i) + noise_ub;
         
       obstacle_bound.ub_head(i) = obstacle_bound.ub_head(i) + noise_ub;
       obstacle_bound.ub_back(i) = obstacle_bound.ub_back(i) + noise_ub;
   end
   
   obstacle_avoid_bound.lb = ...
            [obstacle_avoid_bound.lb_head; obstacle_avoid_bound.lb_back];
   obstacle_avoid_bound.ub = ...    
            [obstacle_avoid_bound.ub_head; obstacle_avoid_bound.ub_back];
end
