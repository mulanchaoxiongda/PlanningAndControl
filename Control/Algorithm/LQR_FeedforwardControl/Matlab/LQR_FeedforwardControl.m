% LQR_FeedforwardControl
% 作者：木兰超凶的
% 日期：20220220
close all;
clear all;
clc;

load  reftraj.mat

%% 相关参数定义
RefTraj = reftraj;       % 参考轨迹点
Control_Period = 0.05;   % 控制周期
L = 2.6;                 % 前后车轮轴距

%% 主程序
x = RefTraj(1,1);% 小车初始状态
y = RefTraj(1,2);
phi = RefTraj(1,3);
v = 0;
wz = 0;
state = [x y phi v wz];

err_state_ini = [0 -0.3 0/57.3 0 0];%小车初始位置偏差
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

    [v_command, delta_f_command, idx_target] = LQR_add_FeedForwardControl(RefTraj, state_measured, Control_Period, L);
        
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
    result_command(i,1) = t;
    result_command(i,2) = delta_f_command;

    i = i+1;
end

result_command(1,1) = 0;
result_command(1,2) = result_command(2,2);

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
plot(result_command(:,1),tan(result_command(:,2))*state(4)/L*57.3,'r',result_state(:,1),result_state(:,6)*57.3,'b'); grid on;
xlabel('时间 / s');
ylabel('横摆角速度 / 度每秒');
legend('横摆角速度指令','真实横摆角速度');

%% 计算控制量:速度和方向盘转角
function [v_command, delta_f_command, idx_target] = LQR_add_FeedForwardControl(RefTraj, state_measured, Control_Period, L)
	SizeOfRefTraj = size(RefTraj,1);
    for i = 1:1:SizeOfRefTraj
        dist(i,1) = norm(RefTraj(i,1:2) - state_measured(1,1:2));   
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
    
    Ref_x = (RefTraj(idx_target,1)*dist2+RefTraj(idx_target+1,1)*dist1)/(dist1+dist2);
    Ref_y = (RefTraj(idx_target,2)*dist2+RefTraj(idx_target+1,2)*dist1)/(dist1+dist2);    
    Ref_Psi = (RefTraj(idx_target,3)*dist2+RefTraj(idx_target+1,3)*dist1)/(dist1+dist2);
    Ref_Cur = (RefTraj(idx_target,5)*dist2+RefTraj(idx_target+1,5)*dist1)/(dist1+dist2);
    Ref_Vel = (RefTraj(idx_target,4)*dist2+RefTraj(idx_target+1,4)*dist1)/(dist1+dist2);
    
    err_x = state_measured(1)-Ref_x;
    err_y = state_measured(2)-Ref_y;
    err_psi = state_measured(3)-Ref_Psi;
    
    X = [err_x err_y err_psi]';
    
    A = [1, 0, -Ref_Vel*Control_Period*sin(Ref_Psi);
         0, 1, Ref_Vel*Control_Period*cos(Ref_Psi);
         0, 0, 1];
     
    B = [Control_Period*cos(Ref_Psi), 0;
         Control_Period*sin(Ref_Psi), 0;
         0, Control_Period];
     
    Q = [3 0 0
        0 5 0
        0 0 8];
    
    R = [1 0
        0 2];
    
    iter_max = 500; % 迭代法求增益矩阵K
    epsilon = 0.01; 
    
    P_old = Q;
    
    for i = 1:iter_max
        P_new = A'*P_old*A-(A'*P_old*B)/(R+B'*P_old*B)*(B'*P_old*A)+Q;
        if abs(P_new-P_old)<=epsilon
            break;
        else
            P_old = P_new;
        end
    end
    
    P = P_new;
    
    K = (B'*P*B+R)\(B'*P*A);

    u_control = -K*X;
    
    CurVel_compensate = state_measured(4)*Ref_Cur; % 左转为正,右转为负
    
    wz_command = u_control(2)+CurVel_compensate;
    if abs(wz_command)>60/57.3
        wz_command = 60/57.3*sign(wz_command);
    end
    
    delta_f_command = atan(wz_command*L/state_measured(4));
    if abs(delta_f_command)>60/57.3
        delta_f_command = 60/57.3*sign(delta_f_command);
    end
    
    v_command = u_control(1)+Ref_Vel;
end

%% 更新小车量测信息
function state_measured = UpdateStateMeasured(state, L)
    gate_noise = 0;
    
    para_k = mod(state(1),1); % 漂移误差;若para_k取1,则为常值误差
    err_measure_x = 0.00*para_k;
    err_measure_y = -0.03*para_k;
    err_measure_psi = -2/57.3*para_k;
    err_measure_v = 0.0;
    err_L = 0.0;
    
    gate_postion_measured = 0; % 0定位点为后轮中心1定位点为车体几何中心
    
    if gate_postion_measured==0
        state_measured(1) = state(1) + gate_noise*err_measure_x;   % 横坐标
        state_measured(2) = state(2) + gate_noise*err_measure_y;   % 纵坐标
        state_measured(3) = state(3) + gate_noise*err_measure_psi; % 横摆角
        state_measured(4) = state(4) + gate_noise*err_measure_v;   % 速度
    elseif gate_postion_measured==1
        state_measured(1) = state(1) + 0.5*(L+err_L)*cos(state(3)) + gate_noise*err_measure_x;  % 横坐标
        state_measured(2) = state(2) + 0.5*(L+err_L)*sin(state(3)) + gate_noise*err_measure_y;  % 纵坐标
        state_measured(3) = state(3) + gate_noise*err_measure_psi;                      % 横摆角
        state_measured(4) = state(4) + gate_noise*err_measure_v;                        % 速度; % 简化处理，忽略角速度引起的线速度
    end
    
    gate_delay = 1; % 定位信息延时开关
    
    if gate_delay==1
        persistent state_measured_storage;
        if isempty(state_measured_storage)
            state_measured_storage=0;
        end
        
        persistent counter;
        if isempty(counter)
            counter=1;
        end
        state_measured_storage(counter,1) = state_measured(1);
        state_measured_storage(counter,2) = state_measured(2);
        state_measured_storage(counter,3) = state_measured(3);
        state_measured_storage(counter,4) = state_measured(4);
        counter = counter+1;
        
        N = 2; % 延迟时间N*Control*dt
        
        if counter>N
            state_measured(1) = state_measured_storage(counter-N,1);
            state_measured(2) = state_measured_storage(counter-N,2);
            state_measured(3) = state_measured_storage(counter-N,3);
            state_measured(4) = state_measured_storage(counter-N,4);
        else
            state_measured(1) = state_measured_storage(1,1);
            state_measured(2) = state_measured_storage(1,2);
            state_measured(3) = state_measured_storage(1,3);
            state_measured(4) = state_measured_storage(1,4);
        end
    end
    
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
    
    err_y_vec = cross(Q1-P,Q2-Q1)/norm(Q2-Q1);
    
    TrackingErrorOfPosition = err_y_vec(3);
    TrackingErrorOfAttitude = state(3)-Ref_Psi;
end

%% 更新小车状态量
function state_new = UpdateState(v_command,delta_f_command,state_old,dt,L)
    Tv = 0.10;
    a = (v_command-state_old(4))/Tv;%变参--与运动速度和载荷相关
    
    persistent delta_f;
    if isempty(delta_f)
        delta_f=0;
    end
    
    err_delta_f0 = 0.5/57.3;
    err_L = 0.02;
    
    Tw = 0.15;
    d_delta_f = (delta_f_command-delta_f)/Tw;
    delta_f = delta_f + d_delta_f*dt;
    
    state_new(1) = state_old(1) + state_old(4)*cos(state_old(3))*dt; %横坐标
    state_new(2) = state_old(2) + state_old(4)*sin(state_old(3))*dt; %纵坐标
    state_new(3) = state_old(3) + tan(delta_f+err_delta_f0)*state_old(4)/(L+err_L)*dt;    %横摆角
    state_new(4) = state_old(4) + a*dt;                              %速度
    state_new(5) = tan(delta_f+err_delta_f0)*state_old(4)/(L+err_L);         %横摆角速度
end
