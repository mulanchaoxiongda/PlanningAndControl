function [ego_speed, simulation_time, simulation_step, ...
        ego_init_state, speed_para, ego_para] = CreatSimulationPara()
    ego_speed = 40 /3.6;
    simulation_time = 8.0;
    simulation_step = 0.1;

    ego_init_state.init_speed = ego_speed;
    ego_init_state.init_acc = 0.0;
    
    speed_para.speed_limit = 40 /3.6;
    speed_para.ego_speed = ego_speed;
    speed_para.keep_clean_min_speed = 5/3.6;
    speed_para.cruise_speed = 30 /3.6;
    speed_para.cruise_state_flag = false;
    speed_para.keep_clean_flag = false;
    
    ego_para.length = 4.7;
    ego_para.width = 2.0;
end