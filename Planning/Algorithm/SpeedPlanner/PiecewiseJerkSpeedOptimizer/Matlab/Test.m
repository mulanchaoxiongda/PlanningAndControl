close all;
clear all;
clc;

load input_info.mat;

% creat input info
[sample_interval, s_bound, ds_bound, dds_bound, ddds_bound, ...
        init_state, end_state, integral_step, heuristic_speed_data, ...
        cruise_speed] = CreatInputInfo();

% path planning algorithm
tic;
[speed_data, opt_soltion] = ...
        PiecewiseJerkSpeedOptimizer(sample_interval, s_bound, ...
        ds_bound, dds_bound, ddds_bound, init_state, end_state, ...
        integral_step, heuristic_speed_data, cruise_speed);
toc;

% result analysis
ResultAnalysis(...
        st_graph, obs_info, opt_st_path, closed_list, ...
        open_list, ego_para, speed_data, planning_para);
    