% Stanley
% 作者：木兰超凶哒
% 日期：20220220
close all;
clear all;
clc;

load  reftraj.mat

%% 相关参数定义
RefTraj = reftraj;       % 参考轨迹点
Control_Period = 0.02;   % 控制周期
L = 2.6;                 % 前后车轮轴距

%% 主程序
x = RefTraj(1,1);% 小车初始状态
y = RefTraj(1,2);
phi = RefTraj(1,3);
v = 0;
wz = 0;
state = [x y phi v wz];
err_state_ini = [0 -1 0/57.3 0 0];%小车初始位置偏差
state = state + err_state_ini;
state_measured = state;

idx_target = 1;

t = 0;
dt = Control_Period;

i = 1;
result_state(i,1) = t;
result_state(i,2) = state(1);
result_state(i,3) = state(2);
result_state(i,4) = state(3);
result_state(i,5) = state(4);
result_state(i,6) = state(5);
[TrackingErrorOfPosition, TrackingErrorOfAttitude] = CalculateTrackingError(RefTraj, state);

result_TrackingError(i,1) = t;
result_TrackingError(i,2) = TrackingErrorOfPosition;
result_TrackingError(i,3) = TrackingErrorOfAttitude;

result_state_measured(i,1) = t;
result_state_measured(i,2) = state_measured(1);
result_state_measured(i,3) = state_measured(2);
result_state_measured(i,4) = state_measured(3);
result_state_measured(i,5) = state_measured(4);

% 循环遍历轨迹点
while idx_target < size(RefTraj,1)-2
    
    t = t+dt;
    
    [TrackingErrorOfPosition, TrackingErrorOfAttitude] = CalculateTrackingError(RefTraj, state);
    
    state_measured = UpdateStateMeasured(state, L);
    
    [v_command, delta_f_command, idx_target] = stanley(RefTraj, state_measured, L);
    
    state = UpdateState(v_command, delta_f_command, state, dt,L);
    
    result_state(i,1) = t;
    result_state(i,2) = state(1);
    result_state(i,3) = state(2);
    result_state(i,4) = state(3);
    result_state(i,5) = state(4);
    result_state(i,6) = state(5);
    result_TrackingError(i,1) = t;
    result_TrackingError(i,2) = TrackingErrorOfPosition;
    result_TrackingError(i,3) = TrackingErrorOfAttitude;
    result_state_measured(i,1) = t;
    
    result_state_measured(i,2) = state_measured(1);
    result_state_measured(i,3) = state_measured(2);
    result_state_measured(i,4) = state_measured(3);
    result_state_measured(i,5) = state_measured(4);

    i = i+1;
    
end

% 画图
figure('name','轨迹')
plot(RefTraj(:,1), RefTraj(:,2), 'b'); grid on;
xlabel('纵坐标 / m');
ylabel('横坐标 / m');
hold on 
for i = 1:size(result_state,1)
    scatter(result_state(i,2), result_state(i,3),80, '.r');
    pause(0.0001);
end
legend('规划轨迹', '实际轨迹');

figure('name','跟踪误差')
subplot(2,1,1);
plot(result_TrackingError(:,1), result_TrackingError(:,2)*100, 'b'); grid on;
xlabel('时间 / s');
ylabel('位置误差 / cm');
subplot(2,1,2);
plot(result_TrackingError(:,1), result_TrackingError(:,3)*57.3, 'b'); grid on;
xlabel('时间 / s');
ylabel('姿态误差 / 度');

figure('name','定位信息')
plot(RefTraj(:,1), RefTraj(:,2), 'b'); grid on;
xlabel('横坐标 / m');
ylabel('纵坐标 / m');
hold on 
plot(result_state_measured(:,2), result_state_measured(:,3),'.r');
legend('规划轨迹', '定位信息');

figure('name','横摆角速度');
plot(result_state(:,1),result_state(:,6)*57.3); grid on;
xlabel('时间 / s');
ylabel('横摆角速度 / 度每秒');

%% 计算控制量:速度和方向盘转角
function [v_command, delta_f_command, idx_target] = stanley(RefTraj, state_measured, L)
    state_frontsteel_measured(1) = state_measured(1)+L*cos(state_measured(3));
    state_frontsteel_measured(2) = state_measured(2)+L*sin(state_measured(3));
    
    SizeOfRefTraj = size(RefTraj,1);
    for i = 1:1:SizeOfRefTraj
        dist(i,1) = norm(RefTraj(i,1:2) - state_frontsteel_measured(1,1:2));
    end
    [~,idx] = min(dist); % 距离最近轨迹点编号
    
    if idx == 1
        idx_target = idx;
    elseif idx == SizeOfRefTraj
        idx_target = idx-1;
    else
        if dist(idx-1)<dist(idx+1)
            idx_target = idx-1;
        else
            idx_target = idx;
        end
    end
    
    dist1 = dist(idx_target);
    dist2 = dist(idx_target+1);
    
    RefPoint = ([RefTraj(idx_target,1) RefTraj(idx_target,2) 0]*dist2+[RefTraj(idx_target+1,1) RefTraj(idx_target+1,2) 0]*dist1)/(dist1+dist2);
    
    Ref_Psi = (RefTraj(idx_target,3)*dist2+RefTraj(idx_target+1,3)*dist1)/(dist1+dist2);
    Ref_Vel = (RefTraj(idx_target,4)*dist2+RefTraj(idx_target+1,4)*dist1)/(dist1+dist2);
    Ref_Cur = (RefTraj(idx_target,5)*dist2+RefTraj(idx_target+1,5)*dist1)/(dist1+dist2); % 左转为正
    
    P = [state_measured(1) state_measured(2) 0];
    Q1 = RefPoint;
    
    while Ref_Psi >= 2*pi
        Ref_Psi = Ref_Psi-2*pi;
    end
    
    while Ref_Psi < 0
        Ref_Psi = Ref_Psi+2*pi;
    end
    
    if Ref_Psi>=0&&Ref_Psi<pi/2
        det_x = 1;
    elseif Ref_Psi>pi/2&&Ref_Psi<pi*3/2
        det_x = -1;
    elseif Ref_Psi>pi*3/2&&Ref_Psi<pi*2
        det_x = 1;
    end
    
    Q2 = [RefPoint(1)+det_x RefPoint(2)+det_x*tan(Ref_Psi) 0];
    
    err_y_vec = cross(Q2-Q1,Q1-P)/norm(Q2-Q1);
    err_y = err_y_vec(3);
    err_psi = Ref_Psi-state_measured(3);
    
    k = 0.5;
    delta_y_command = atan(k * err_y / state_measured(4));
    delta_psi_command = err_psi;
    
    delta_f_command = delta_y_command+delta_psi_command; % 曲率补偿效果不好，因为转向中心在后轮中心，Stanley算法的控制中心在前轮中点
    v_command = Ref_Vel;
end

%% 更新小车量测信息
function state_measured = UpdateStateMeasured(state, L)
        state_measured(1) = state(1); % 横坐标
        state_measured(2) = state(2); % 纵坐标
        state_measured(3) = state(3); % 横摆角
        state_measured(4) = state(4); % 速度    
end

%% 计算控跟踪误差:位置误差和姿态误差
function [TrackingErrorOfPosition, TrackingErrorOfAttitude] = CalculateTrackingError(RefTraj, state)
	SizeOfRefTraj = size(RefTraj,1);
    for i = 1:1:SizeOfRefTraj
        dist(i,1) = norm(RefTraj(i,1:2) - state(1,1:2));   
    end
    [~,idx] = min(dist); % 距离最近轨迹点编号
    
    if idx == 1
        idx_target = idx;
    elseif idx == SizeOfRefTraj
        idx_target = idx-1;
    else
        if dist(idx-1)<dist(idx+1)
            idx_target = idx-1;
        else
            idx_target = idx;
        end
    end
    
    dist1 = dist(idx_target);
    dist2 = dist(idx_target+1);
    
    RefPoint = ([RefTraj(idx_target,1) RefTraj(idx_target,2) 0]*dist2+[RefTraj(idx_target+1,1) RefTraj(idx_target+1,2) 0]*dist1)/(dist1+dist2);
    Ref_Psi = (RefTraj(idx_target,3)*dist2+RefTraj(idx_target+1,3)*dist1)/(dist1+dist2);
    
    P = [state(1) state(2) 0];
    Q1 = RefPoint;
    
    while Ref_Psi > 2*pi
        Ref_Psi = Ref_Psi-2*pi;
    end
    while Ref_Psi < -2*pi
        Ref_Psi = Ref_Psi+2*pi;
    end
    
    if Ref_Psi>=0&&Ref_Psi<pi/2
        det_x = 1;
    elseif Ref_Psi>pi/2&&Ref_Psi<pi*3/2
        det_x = -1;
    elseif Ref_Psi>pi*3/2&&Ref_Psi<pi*2
        det_x = 1;
    end
    
    Q2 = [RefPoint(1)+det_x RefPoint(2)+det_x*tan(Ref_Psi) 0];
    
    err_y_vec = cross(Q1-P,Q2-Q1)/norm(Q2-Q1);
    
    TrackingErrorOfPosition = err_y_vec(3);
    TrackingErrorOfAttitude = state(3)-Ref_Psi;
end

%% 更新小车状态量
function state_new = UpdateState(v_command,delta_f_command,state_old,dt,L)
    Tv = 0.10; a = (v_command-state_old(4))/Tv;
    
    persistent delta_f;
    if isempty(delta_f)
        delta_f=0;
    end
    err_delta_f0 = 0.5/57.3;
    err_L = 0.02;
    Tw = 0.15; d_delta_f = (delta_f_command-delta_f)/Tw; delta_f = delta_f + d_delta_f*dt;
    
    state_new(1) = state_old(1) + state_old(4)*cos(state_old(3))*dt;                   % 横坐标
    state_new(2) = state_old(2) + state_old(4)*sin(state_old(3))*dt;                   % 纵坐标
    state_new(3) = state_old(3) + tan(delta_f+err_delta_f0)*state_old(4)/(L+err_L)*dt; % 横摆角
    state_new(4) = state_old(4) + a*dt;                                                % 速度
    state_new(5) = tan(delta_f+err_delta_f0)*state_old(4)/(L+err_L);                   % 横摆角速度
end
    