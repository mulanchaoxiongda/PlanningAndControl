function [st_graph] = CreatPathTimeGraph( ...
        ref_speed, planning_cfg, sl_path_data, speed_para, simulation_time)
    total_time = simulation_time;
    total_s = (total_time + 0.0001) * ref_speed; % ref_speed = max(ego_speed, speed_limit, cruise_speed)
    
    delta_t = 0.5; % 取0.5s,AStar调参难迎刃而解，搜索空间完备性大幅提高，粗规划路径品质也好很多

    st_graph.total_time = total_time;
    st_graph.total_s = total_s;
    st_graph.dt = delta_t;
    st_graph.t_dividing = 3.0; % 整数
    st_graph.t = [0 : delta_t : total_time]';
    st_graph.s = [];
    st_graph.cost = [];
    st_graph.kappa = []; % Todo:求解
    st_graph.speed_limit = []; % Todo:求解
    
    s = 0;
    while s < total_s
        st_graph.s = [st_graph.s; s];
        
        kappa = 0.02; % kappa = f(s)
% % %         if s > total_s * 0.5
% % %             kappa = 2.5 / (speed_para.cruise_speed * 0.6)^2;
% % %         end
        st_graph.kappa = [st_graph.kappa; kappa];
        
        if s < ref_speed * st_graph.t_dividing
            jerk_resolution = 3; % 结合planning.para & jerk_min&planning.para.jerk_max取值
        else
            jerk_resolution = 5;
        end
        
        ds = 1 /6 * jerk_resolution * delta_t^3;
        s  = s + ds;
    end

    idx_left = 1;
    idx_right = length(st_graph.s);
    
    while idx_left < idx_right
        temp = st_graph.s(idx_right);
        
        st_graph.s(idx_right) = st_graph.s(idx_left);
        st_graph.s(idx_left) = temp;
        
        temp = st_graph.kappa(idx_right);
        
        st_graph.kappa(idx_right) = st_graph.kappa(idx_left);
        st_graph.kappa(idx_left) = temp;
        
        idx_left = idx_left + 1;
        idx_right = idx_right - 1;
    end

    st_graph.speed_limit = ...
            ones(length(st_graph.kappa)) * speed_para.speed_limit; % 交规限速 && 曲率限速
        
    for i = 1 : 1 : length(st_graph.s)
        cost = [];
        for j = 1 : 1 : length(st_graph.t)
            cost = [cost, Inf];
        end
        st_graph.cost = [st_graph.cost; cost];
    end

    st_graph.s_dense = [];
    st_graph.kappa_dense = []; % Todo:求解
    
    st_graph.s_dense = st_graph.s;
    st_graph.kappa_dense = st_graph.kappa;
    
    st_graph.speed_limit_dense = ...
            ones(length(st_graph.kappa_dense)) * speed_para.speed_limit;
        
    st_graph.s_sparse = [];
    st_graph.kappa_sparse = []; % Todo:求解
    
    s=0;
    while s <= total_s
        st_graph.s_sparse = [st_graph.s_sparse; s];

        kappa = 0.02;
% % %         if s > total_s * 0.5
% % %             kappa = 2.5 * (speed_para.cruise_speed * 0.6)^2;
% % %         end
        st_graph.kappa_sparse = [st_graph.kappa_sparse; kappa];

        jerk_resolution = 5;

        ds = 1 / 6 * jerk_resolution * delta_t^3;
        s = s + ds;
    end

    idx_left = 1;
    idx_right = length(st_graph.s_sparse);
    
    while idx_left < idx_right
        temp = st_graph.s_sparse(idx_right);

        st_graph.s_sparse(idx_right) = st_graph.s_sparse(idx_left);
        st_graph.s_sparse(idx_left) = temp;
        
        temp = st_graph.kappa_sparse(idx_right);
        
        st_graph.kappa_sparse(idx_right) = st_graph.kappa_sparse(idx_left);
        st_graph.kappa_sparse(idx_left) = temp;

        idx_left = idx_left + 1;
        idx_right = idx_right - 1;
    end

    st_graph.speed_limit_sparse = ...
        ones(length(st_graph.kappa_sparse)) * speed_para.speed_limit;

    st_graph.total_s = st_graph.s_sparse(1);
end
