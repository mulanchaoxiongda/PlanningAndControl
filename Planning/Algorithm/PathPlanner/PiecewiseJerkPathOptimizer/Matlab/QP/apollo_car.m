% apollo_car机动能力挖掘空间分析
clear all;
close all;
clc;

%% vehicle para
L = 2.8448;
steer_ratio = 16;
steer_max = 8.2;
steer_rate_max = 7.0;

%% 2km/h
v = 2 / 3.6;
R = L / tan(steer_max / steer_ratio);

steer = atan(L / R) * steer_ratio;
steer_rate = steer_rate_max / 2;

w = v / R;
ay = v^2 / R;

i = 1;
result(i, 1) = v;
result(i, 2) = steer;
result(i, 3) = steer_rate;
result(i, 4) = R;
result(i, 5) = 0;
result(i, 6) = steer_max;
result(i, 7) = steer_rate_max;
result(i, 8) = w;
result(i, 9) = ay;

%% 5km/h
v = 5 / 3.6;
R = L / tan(steer_max / steer_ratio) * 1.1;

steer = atan(L / R) * steer_ratio;
steer_rate = steer_rate_max / 2;

w = v / R;
ay = v^2 / R;

i = i + 1;
result(i, 1) = v;
result(i, 2) = steer;
result(i, 3) = steer_rate;
result(i, 4) = R;
result(i, 5) = 0;
result(i, 6) = steer_max;
result(i, 7) = steer_rate_max;
result(i, 8) = w;
result(i, 9) = ay;

%% 10km/h
v = 10 / 3.6;
R = L / tan(steer_max / steer_ratio) * 1.58;

steer = atan(L / R) * steer_ratio;
steer_rate = steer_rate_max / 2;

w = v / R;
ay = v^2 / R;

i = i + 1;
result(i, 1) = v;
result(i, 2) = steer;
result(i, 3) = steer_rate;
result(i, 4) = R;
result(i, 5) = 0;
result(i, 6) = steer_max;
result(i, 7) = steer_rate_max;
result(i, 8) = w;
result(i, 9) = ay;

%% 20km/h
v = 20 / 3.6;
R = L / tan(steer_max / steer_ratio) * 3.1;

steer = atan(L / R) * steer_ratio;
steer_rate = steer_rate_max / 2;

w = v / R;
ay = v^2 / R;

i = i + 1;
result(i, 1) = v;
result(i, 2) = steer;
result(i, 3) = steer_rate;
result(i, 4) = R;
result(i, 5) = 0;
result(i, 6) = steer_max;
result(i, 7) = steer_rate_max;
result(i, 8) = w;
result(i, 9) = ay;

%% 40km/h
v = 40 / 3.6;
R = L / tan(steer_max / steer_ratio) * 8.3;

steer = atan(L / R) * steer_ratio;
steer_rate = steer_rate_max / 2;

w = v / R;
ay = v^2 / R;

i = i + 1;
result(i, 1) = v;
result(i, 2) = steer;
result(i, 3) = steer_rate;
result(i, 4) = R;
result(i, 5) = 0;
result(i, 6) = steer_max;
result(i, 7) = steer_rate_max;
result(i, 8) = w;
result(i, 9) = ay;

%% 60km/h
v = 60 / 3.6;
R = L / tan(steer_max / steer_ratio) * 18.5;

steer = atan(L / R) * steer_ratio;
steer_rate = steer_rate_max / 2;

w = v / R;
ay = v^2 / R;

i = i + 1;
result(i, 1) = v;
result(i, 2) = steer;
result(i, 3) = steer_rate;
result(i, 4) = R;
result(i, 5) = 0;
result(i, 6) = steer_max;
result(i, 7) = steer_rate_max;
result(i, 8) = w;
result(i, 9) = ay;

%% 80km/h
v = 80 / 3.6;
R = L / tan(steer_max / steer_ratio) * 33;

steer = atan(L / R) * steer_ratio;
steer_rate = steer_rate_max / 2;

w = v / R;
ay = v^2 / R;

i = i + 1;
result(i, 1) = v;
result(i, 2) = steer;
result(i, 3) = steer_rate;
result(i, 4) = R;
result(i, 5) = 0;
result(i, 6) = steer_max;
result(i, 7) = steer_rate_max;
result(i, 8) = w;
result(i, 9) = ay;

%% 120km/h
v = 120 / 3.6;
R = L / tan(steer_max / steer_ratio) * 73;

steer = atan(L / R) * steer_ratio;
steer_rate = steer_rate_max / 2;

w = v / R;
ay = v^2 / R;

i = i + 1;
result(i, 1) = v;
result(i, 2) = steer;
result(i, 3) = steer_rate;
result(i, 4) = R;
result(i, 5) = 0;
result(i, 6) = steer_max;
result(i, 7) = steer_rate_max;
result(i, 8) = w;
result(i, 9) = ay;

%% 140km/h
v = 140 / 3.6;
R = L / tan(steer_max / steer_ratio) * 100;

steer = atan(L / R) * steer_ratio;
steer_rate = steer_rate_max / 2;

w = v / R;
ay = v^2 / R;

i = i + 1;
result(i, 1) = v;
result(i, 2) = steer;
result(i, 3) = steer_rate;
result(i, 4) = R;
result(i, 5) = 0;
result(i, 6) = steer_max;
result(i, 7) = steer_rate_max;
result(i, 8) = w;
result(i, 9) = ay;

%% plot
figure('name', 'apollo_car'); 
subplot(2, 2, 1);
plot(result(:, 1) * 3.6, result(:, 2) *57.3 / steer_ratio, '*-b');
xlabel('车速(km/h)'); ylabel('前轮转角(度)');
title('车速 -- 前轮转角');
 
subplot(2, 2, 2);
plot(result(:, 1) * 3.6, result(:, 4), '*-b');
xlabel('车速(km/h)'); ylabel('转弯半径(m)');
title('车速 -- 转弯半径');
 
subplot(2, 2, 3);
plot(result(:, 1) * 3.6, result(:, 8) *57.3, '*-b');
xlabel('车速(km/h)'); ylabel('车辆横摆角速度(度/秒)');
title('车速 -- 车辆横摆角速度');
 
subplot(2, 2, 4);
plot(result(:, 1) * 3.6, result(:, 9), '*-b');
xlabel('车速(km/h)'); ylabel('车辆离心加速度(米/秒方)');
title('车速 -- 车辆离心加速度'); 
