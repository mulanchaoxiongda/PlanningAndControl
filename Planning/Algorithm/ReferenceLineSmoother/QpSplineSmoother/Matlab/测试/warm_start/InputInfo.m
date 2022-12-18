close all;
clear all;
clc;

%% 读入路径数据[x, y, theta]
% ref_path = load('refPath.mat'); % Hybrid Astar生成的路径点，相邻两点间距1cm
ref_path = load('refPath.mat'); % Hybrid Astar生成的路径点，相邻两点间距1cm
% ref_path = ref_path.refPath(1:699, 1 : 3);
ref_path = ref_path.refPath(1:699, 1 : 3);

% N = 801; % 直角转弯轨迹，相邻两点间距1cm
% for i = 1 : 1 : 261
%     ref_path(i, 1) = (i - 1) / 100;
%     ref_path(i, 2) = 0;
%     ref_path(i, 3) = 0;
% end
% 
% for i = 262 : 1 : 801
%     ref_path(i, 1) = (261 - 1) / 100;
%     ref_path(i, 2) = (i - 261) / 100;
%     ref_path(i, 3) = pi / 2;
% end

gate_noise = 0; % 噪声开关----routing_line点位置无噪声
for i = 1 : 1 : size(ref_path, 1)
    ref_path(i, 1) = ref_path(i, 1) + normrnd(0, 0.01) * gate_noise;
    ref_path(i, 2) = ref_path(i, 2) + normrnd(0, 0.01) * gate_noise;
    ref_path(i, 3) = ref_path(i, 3) + normrnd(0, 0.5 / 57.3) * gate_noise;
end

% plot(ref_path(:, 1), ref_path(:, 2), 'b'); axis equal; hold on;

interval = 20; % 路径点间距 interval cm

routing_line.x = ref_path(1: interval : size(ref_path, 1), 1);
routing_line.y = ref_path(1: interval : size(ref_path, 1), 2);
routing_line.theta = ref_path(1: interval : size(ref_path, 1), 3);

% plot(routing_line.x(:), routing_line.y(:), 'y'); hold on; axis equal;

% tic;
[ref_line] = MySmoothRefLine(routing_line);
% toc;

% 
% figure();
% plot(ref_line.x(:), ref_line.y(:), 'b'); axis equal;
