% Purepursuit
% 作者：木兰超凶哒
% 日期：20220220
close all;
clear all;
clc;


%% 读入参考轨迹
load  reftraj.mat

refTraj = reftraj;


%% 参数设置
L = 2.6; % 前后车轮轴距

controlPeriod = 0.05;


%% 初始条件设置
x = refTraj(1, 1); % 小车初始状态
y = refTraj(1, 2);
phi = refTraj(1, 3);
v = 0;

state = [x y phi v];
errStateInit = [0 -1 -5 / 57.3 0]; % 初始偏差
state = state + errStateInit;


%% 主程序
idxRefPoint = 1;

t = 0;
dt = controlPeriod;

i = 1;
resultState(i, 1) = t;
resultState(i, 2) = state(1);
resultState(i, 3) = state(2);
resultState(i, 4) = state(3);
resultState(i, 5) = state(4);

[trackingErrorOfPosition, trackingErrorOfAttitude] = calculateTrackingError(refTraj, state);

resultTrackingError(i, 1) = t;
resultTrackingError(i, 2) = trackingErrorOfPosition;
resultTrackingError(i, 3) = trackingErrorOfAttitude;

% 循环遍历轨迹点
while idxRefPoint < size(refTraj, 1) - 2
    
    t = t+dt;
    
    [trackingErrorOfPosition, trackingErrorOfAttitude] = calculateTrackingError(refTraj, state);
    
    stateMeasured = updateStateMeasured(state);
      
    [speedCommand, deltaFrontCommand, idxRefPoint] = purePursuitControl(refTraj, stateMeasured, L);
    
    state = updateState(speedCommand,deltaFrontCommand, state, dt, L);
    
    resultState(i, 1) = t;
    resultState(i, 2) = state(1);
    resultState(i, 3) = state(2);
    resultState(i, 4) = state(3);
    resultState(i, 5) = state(4);
    
    resultTrackingError(i, 1) = t;
    resultTrackingError(i, 2) = trackingErrorOfPosition;
    resultTrackingError(i, 3) = trackingErrorOfAttitude;

    i = i+1;
    
    trackingErrorOfPosition;

end

% 画图
figure('name', '轨迹')
plot(refTraj(:, 1), refTraj(:, 2), 'b'); grid on;
xlabel('纵坐标 / m'); ylabel('横坐标 / m'); hold on;

for i = 1 : size(resultState, 1)
    scatter(resultState(i, 2), resultState(i, 3), 80, '.r');
    pause(0.0001);
end
legend('规划轨迹', '实际轨迹');

figure('name', '跟踪误差')
subplot(2, 1, 1);
plot(resultTrackingError(:, 1), resultTrackingError(:, 2) * 100, 'b'); grid on;
xlabel('时间 / s'); ylabel('位置误差 / cm');

subplot(2, 1, 2);
plot(resultTrackingError(:, 1), resultTrackingError(:, 3) * 57.3, 'b'); grid on;
xlabel('时间 / s');
ylabel('姿态误差 / 度');


%% 计算控制量:速度和方向盘转角
function [speedCommand,deltaFrontCommand, idxRefPoint] = purePursuitControl(refTraj, stateMeasured, L)
    lengthOfRefTraj = size(refTraj,1);
    for i = 1:1:lengthOfRefTraj
        dist(i,1) = norm(refTraj(i,1:2) - stateMeasured(1,1:2));   
    end
    [~,idx] = min(dist); % 距离最近轨迹点索引值
    
    Kv = 0.1;   % 前视距离系数
    Ld0 = 0.7; % Ld0是预瞄距离的下限值
    Ld = Kv*stateMeasured(4) + Ld0;
    
    lengthTraj = 0; % 轨迹线长度
    while lengthTraj < Ld && idx < lengthOfRefTraj
        lengthTraj = lengthTraj + norm(refTraj(idx+1, 1 : 2) - refTraj(idx, 1 : 2));
        idx = idx + 1;
    end
    idxRefPoint = idx; % 预瞄轨迹点索引值
    
    lookaheadPoint = refTraj(idxRefPoint, :);
    alpha = atan2(lookaheadPoint(1, 2) - stateMeasured(1, 2), lookaheadPoint(1, 1) - stateMeasured(1, 1))  - stateMeasured(1, 3);

   deltaFrontCommand = atan2(2 * L * sin(alpha), Ld);
   
   speedCommand = lookaheadPoint(1, 4);
end

%% 更新小车量测信息
function stateMeasured = updateStateMeasured(state)
    gateNoise = 0;
    
    errMeasureX = 0.0;
    errMeasureY = 0.0;
    errMeasurePsi = 0.0;
    errMeasureV = 0.0;
    
    stateMeasured(1) = state(1) + gateNoise*errMeasureX;
    stateMeasured(2) = state(2) + gateNoise*errMeasureY;
    stateMeasured(3) = state(3) + gateNoise*errMeasurePsi;
    stateMeasured(4) = state(4) + gateNoise*errMeasureV;
end

%% 计算控跟踪误差:位置误差和姿态误差
function [trackingErrorOfPosition, trackingErrorOfAttitude] = calculateTrackingError(refTraj, state)
	lengthOfRefTraj = size(refTraj,1);
    for i = 1:1:lengthOfRefTraj
        dist(i,1) = norm(refTraj(i,1:2) - state(1,1:2));   
    end
    [~,idx] = min(dist); % 距离最近轨迹点索引值
    
    if idx == 1
        idxRefPoint = idx;
    elseif idx == lengthOfRefTraj
        idxRefPoint = idx-1;
    else
        if dist(idx - 1)<dist(idx + 1)
            idxRefPoint = idx - 1;
        else
            idxRefPoint = idx;
        end
    end
    
    dist1 = dist(idxRefPoint);
    dist2 = dist(idxRefPoint + 1);
    refPoint = ([refTraj(idxRefPoint, 1) refTraj(idxRefPoint, 2) 0]*dist2 + [refTraj(idxRefPoint + 1, 1) refTraj(idxRefPoint + 1, 2) 0] * dist1) / (dist1 + dist2);
    refPsi = (refTraj(idxRefPoint, 3) * dist2 + refTraj(idxRefPoint + 1, 3) * dist1) / (dist1 + dist2);
    
    posePointP = [state(1) state(2) 0];
    posePointQ1 = refPoint;
    
    while refPsi > 2 * pi
        refPsi = refPsi - 2 * pi;
    end
    while refPsi < -2 * pi
        refPsi = refPsi + 2 * pi;
    end
    
    if refPsi >= 0 && refPsi < pi / 2
        detX = 1;
    elseif refPsi > pi / 2 && refPsi<pi * 3 / 2
        detX = -1;
    elseif refPsi > pi * 3 / 2 && refPsi < pi * 2
        detX = 1;
    end
    
    posePointQ2 = [refPoint(1) + detX, refPoint(2) + detX * tan(refPsi), 0];
    
    errYVec = cross(posePointQ1 - posePointP, posePointQ2 - posePointQ1) / norm(posePointQ2 - posePointQ1);
    
    trackingErrorOfPosition = errYVec(3);
    trackingErrorOfAttitude = state(3)-refPsi;
end

%% 更新小车状态量
function stateNew = updateState(speedCommand, deltaFrontCommand, stateOld,dt, L)
    Tv = 0.10;
    a = (speedCommand-stateOld(4)) / Tv;

    persistent delta_f;
    if isempty(delta_f)
        delta_f = 0;
    end
    errDeltaFront0 = 0.5 / 57.3;
    Tw = 0.15;
    dDeltaFront = (deltaFrontCommand - delta_f) / Tw; delta_f = delta_f + dDeltaFront * dt;

    stateNew(1) = stateOld(1) + stateOld(4) * cos(stateOld(3)) * dt;
    stateNew(2) = stateOld(2) + stateOld(4) * sin(stateOld(3)) * dt;
    stateNew(3) = stateOld(3) + tan(delta_f + errDeltaFront0) * stateOld(4) / L * dt;
    stateNew(4) = stateOld(4) + a * dt;
end
    