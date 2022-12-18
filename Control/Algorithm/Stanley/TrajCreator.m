% 生成轨迹点
% 作者：木兰超凶哒
% 日期：20220222
clear all;
clc;

gate = 0; % 轨迹:0直线;1圆

v0 = 3;
v = v0;
x = 0;
y = 0;
psi = 0/57.3;
cur = 0;

t = 0;
T = 5;
dt = 0.1;
N = T/dt+1;

i = 1;
reftraj(i, 1) = x;
reftraj(i, 2) = y;
reftraj(i, 3) = psi;
reftraj(i, 4) = v;
reftraj(i, 5) = cur;
reftraj(i, 6) = t;

for i = 2 : 1 : N
    
    t = t + dt;
    
    if gate == 0
        x = x + v * cos(psi) * dt;
        y = y + v * sin(psi) * dt;
        psi = 0 / 57.3;
        v = v0;
        cur = 1 / 100000000;
        
    elseif gate == 1
        R = 5;
        v = v0;
        psi = v / R * t;
        x = 0 + R * cos(psi - pi / 2);
        y = 5 + R * sin(psi - pi / 2);
        cur = 1 / R;
    end
    
    reftraj(i, 1) = x;
    reftraj(i, 2) = y;
    reftraj(i, 3) = psi;
    reftraj(i, 4) = v;
    reftraj(i, 5) = cur;
    reftraj(i, 6) = t;

end

reftraj(1, 5) = reftraj(2, 5);

save reftraj reftraj;
