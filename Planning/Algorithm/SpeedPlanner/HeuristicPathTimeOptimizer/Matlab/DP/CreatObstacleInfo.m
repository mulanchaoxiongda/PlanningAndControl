function [obs_info] = CreatObstacleInfo( ...
        simulation_time, simulation_step, path_data, ego_para)
    has_obs = true;
    if has_obs == false
        obs_info.st_overlap = [];
        obs_info.obs_type = 2;
    end
    
    dt = simulation_step;
    ref_traj = path_data;

    %% 障碍物1
    obs_para.length = 4.7;
    obs_para.width = 2.0;
    
    obs_traj = [];
    obs_point.t = 0;
    obs_point.x = 12 + 1000 * 0 - 7 * 1 + 30;
    obs_point.y = 7 + 11.8 * 1;
    obs_point.theta = -pi /4;
    obs_point.s = 0;
    obs_traj = [obs_traj; obs_point];

    obs_speed = 20 /3.6;

    dt = simulation_step;
    while obs_point.t <= simulation_time
        obs_point.t = obs_point.t + dt;
        obs_point.x = obs_point.x + obs_speed * cos(obs_point.theta) * dt;
        obs_point.y = obs_point.y + obs_speed * sin(obs_point.theta) * dt;
        obs_point.theta = obs_point.theta;
        obs_point.s = obs_point.s + obs_speed * dt;

        obs_traj = [obs_traj; obs_point];
    end

    [obs_info_1] = MappingObsToSTGraph(obs_traj, ref_traj, obs_para, ego_para);
    
    obs_info = obs_info_1;

    %% 障碍物2
    obs_para.length = 4.7;
    obs_para.width = 2.0;
    
    obs_traj = [];
    obs_point.t = 0;
    obs_point.x = 12 + 1000 * 0 - 7 * 1;
    obs_point.y = 7 + 11.8 * 1;
    obs_point.theta = -pi /4;
    obs_point.s = 0;
    obs_traj = [obs_traj; obs_point];
    
    obs_speed = 20/3.6;
    
    dt = simulation_step;
    while obs_point.t < simulation_time
        obs_point.t = obs_point.t + dt;
        obs_point.x = obs_point.x + obs_speed * cos(obs_point.theta) * dt;
        obs_point.y = obs_point.y + obs_speed * sin(obs_point.theta) * dt;
        obs_point.theta = obs_point.theta;
        obs_point.s = obs_point.s + obs_speed * dt;

        obs_traj = [obs_traj; obs_point];
    end
    
    [obs_info_2] = MappingObsToSTGraph(obs_traj, ref_traj, obs_para, ego_para);
    obs_info = [obs_info; obs_info_2];

    %%障碍物3
    obs_para.length = 4.7;
    obs_para.width = 2.0;

    obs_traj = [];
    obs_point.t = 0;
    obs_point.x = 35;
    obs_point.y = 15;
    obs_point.theta = -pi /4;
    obs_point.s = 0;
    obs_traj = [obs_traj; obs_point];
    obs_speed = 10 /3.6;
    while obs_point.t < simulation_time
        obs_point.t = obs_point.t + dt;
        obs_point.x = obs_point.x + obs_speed * cos(obs_point.theta) * dt;
        obs_point.y = obs_point.y + obs_speed * sin(obs_point.theta) * dt;
        obs_point.theta = obs_point.theta;
        obs_point.s = obs_point.s + obs_speed * dt;
        
        obs_traj = [obs_traj; obs_point];
    end
    
    [obs_info_3] = MappingObsToSTGraph(obs_traj, ref_traj, obs_para, ego_para);
    
    obs_info = [obs_info; obs_info_3];
end
