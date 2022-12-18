close all;
clear all;
clc;

tic;
 
load  reftraj.mat

%% 相关参数定义
RefTraj = reftraj;       % 参考轨迹点
Control_Period = 0.02;   % 控制周期
L = 2.6;                 % 前后车轮轴距

%% 主程序
x = RefTraj(1,1);% 小车初始状态
y = RefTraj(1,2);
phi = RefTraj(1,3);
v = RefTraj(1,4);
wz = RefTraj(1,4)*RefTraj(1,5);

state = [x y phi v wz];

err_state_ini = [0 -0.015 0/57.3 0 0];%小车初始位置偏差
state = state + err_state_ini;
state_measured = state;

idx_target = 1;

t = 0;
dt = 0.01;
T = RefTraj(end,6)+0.5;

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

while t<=T
    [TrackingErrorOfPosition, TrackingErrorOfAttitude] = CalculateTrackingError(RefTraj, state);
   
    if mod(i-1,Control_Period/dt)==0
        state_measured = UpdateRobotStateMeasured(state, L);
        [v_command, delta_f_command, idx_target] = PID_add_FeedForwardControl(RefTraj, state_measured, Control_Period, L, t);
    end
    
    state = UpdateRobotState(v_command, delta_f_command, state, dt,L);
   
    t = t+dt;
    
    i = i+1;
    
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
    result_command(i,3) = tan(delta_f_command)*result_state_measured(i,5)/L;
    result_command(i,4) = v_command;
    result_command(i,5) = result_state(i-1,2);
end

result_command(1,1) = 0;
result_command(1,2) = result_command(2,2);
result_command(1,3) = result_command(2,3);
result_command(1,4) = result_command(2,4);
result_command(1,5) = result_command(2,5);

%% 画图
figure('name','仿真结果');
subplot(3,3,1);
plot(RefTraj(:,1), RefTraj(:,2), 'b'); grid on;
hold on 
for i = 1:size(result_state,1)
    scatter(result_state(i,2), result_state(i,3),80, '.r');
end
xlabel('纵坐标 / m'); ylabel('横坐标 / m'); legend('规划轨迹', '实际轨迹'); title('轨迹');
subplot(3,3,3);
plot(result_TrackingError(:,1), result_TrackingError(:,2)*100, 'b'); grid on;
xlabel('时间 / s'); ylabel('位置误差 / cm'); title('跟踪误差--位置');
subplot(3,3,4);
plot(result_TrackingError(:,1), result_TrackingError(:,3)*57.3, 'b'); grid on;
xlabel('时间 / s'); ylabel('姿态误差 / 度'); title('跟踪误差--横摆角');
subplot(3,3,5);
plot(result_state_measured(:,2), result_state_measured(:,3),'.r',RefTraj(:,1), RefTraj(:,2), 'b'); grid on;
xlabel('横坐标 / m'); ylabel('纵坐标 / m'); legend('规划轨迹', '定位信息'); title('定位信息');
subplot(3,3,2);
plot(result_command(:,1),result_command(:,3)*57.3,'r',result_state(:,1),result_state(:,6)*57.3,'b',RefTraj(:,6),RefTraj(:,4).*RefTraj(:,5)*57.3,'g'); grid on;
xlabel('时间 / s'); ylabel('横摆角速度 / 度每秒'); legend('指令','真实','规划'); title('横摆角速度');
subplot(3,3,6);
plot(result_command(:,1),result_command(:,2)*57.3); grid on;
xlabel('时间 / s'); ylabel('方向盘转角指令 / 度'); title('方向盘转角指令');
subplot(3,3,7);
plot(result_state(:,1),result_state(:,2),'b',RefTraj(:,6),RefTraj(:,1),'r');
xlabel('时间(秒)'); ylabel('横向位置(米)'); title('横向位置--时间'); legend('实际值','规划值');
subplot(3,3,8);
plot(result_state(:,2),result_state(:,5),'b',RefTraj(:,1),RefTraj(:,4),'r',result_command(:,5),result_command(:,4),'g');
xlabel('横向位置(米)'); ylabel('速度(米/秒)'); title('速度--横向位置'); legend('实际值','规划值','指令值');
subplot(3,3,9);
plot(result_state(:,1),result_state(:,5),'b',RefTraj(:,6),RefTraj(:,4),'r',result_command(:,1),result_command(:,4),'g');
xlabel('时间(秒)'); ylabel('速度(米/秒)'); title('速度--时间'); legend('实际值','规划值','指令值');
toc;
