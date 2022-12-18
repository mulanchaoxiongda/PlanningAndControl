% close all;
clear all;
clc;

% 创建停车场景
scenario = sceneCreatorFunc();

% 配置参数
paraCfg = paraConfigFunc(scenario);

% 创建栅格地图
gridMap = gridMapCreatorFunc(scenario, paraCfg);

% 计算全局引导启发阵
% hCostMatrix = calHeuristicItemFunc(paraCfg.goalPose, gridMap);
load('hCostMatrix'); % 调试程序用

% 规划轨迹序列
[refPath, openList, closedList] = hybridAStarFunc(scenario.startPose, paraCfg.goalPose, scenario, paraCfg, gridMap, hCostMatrix);
% refPath = []; openList = []; closedList = []; % 调试场景使用

% 画图
if isempty(refPath)
    disp("Failed to find path!");
    failedAnalysisFunc(scenario, paraCfg, gridMap, refPath, openList, closedList);
else
    visualizationFunc(scenario, paraCfg, gridMap, refPath, openList, closedList);
end
