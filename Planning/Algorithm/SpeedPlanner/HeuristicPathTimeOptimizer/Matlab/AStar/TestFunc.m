close all;
clear all;
clc;

[ego_speed, simulation_time, simulation_step, ...
        ego_init_state, speed_para, ego_para] = CreatSimulationPara();

tic;
disp('time of creat path time graph :');
[st_graph] = ...
        CreatPathTimeGraph(max(ego_speed, max(speed_para.cruise_speed, ...
        speed_para.speed_limit)), [], [], speed_para, simulation_time);
toc;

[ref_traj] = CreatPathData(simulation_time, ego_speed);

tic;
disp('time of creat obscatle info :');
[obs_info] = ...
        CreatObstacleInfo(simulation_time, simulation_step, ...
        ref_traj, ego_para);
toc;

tic;
disp('time of heuristic search :')
[opt_st_path, closed_list, open_list, planning_para] = ...
        HeuristicSearchFunc(speed_para, ego_init_state, obs_info, st_graph);
toc;

tic;
disp('time of plot simulation result :');
ResultAnalysis(st_graph, obs_info, opt_st_path, ...
               closed_list, open_list, ego_para, planning_para);
toc;

save input_info.mat;


