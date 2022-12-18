% 生成轨迹点
clear all;
clc;

gate = 0; % 轨迹:0直线;1圆

t = 0;
T = 2.5;
dt = 0.05;
N = T/dt+1;

v0 = 0.5*0.15*(t-T)^2;
v = v0;
x = 0;
y = 0;
psi = 0/57.3;
cur = 0;
a = 0;
R0 = 12;
Re = 8;
R = R0;

i = 1;
reftraj(i,1) = x;
reftraj(i,2) = y;
reftraj(i,3) = psi;
reftraj(i,4) = v;
reftraj(i,5) = cur;
reftraj(i,6) = t;

for i=2:1:N
    
    t = t + dt;
    
    if gate==0
        v = 0.5*0.15*(t-T)^2;
        a = 0.15*(t-T);
        x = x+v*cos(psi)*dt;
        y = y+v*sin(psi)*dt;
        psi = 0/57.3;
        cur = 1/100000000;
    elseif gate==1
        R = Re+(t-T)/T*(Re-R0);
        v = 0.5*0.15*(t-T)^2;
        a = 0.15*(t-T);
        psi = psi + v/R*dt;
        x = 0+R*cos(psi-pi/2);
        y = R+R*sin(psi-pi/2);
        cur = 1/R;
    end
    reftraj(i,1) = x;
    reftraj(i,2) = y;
    reftraj(i,3) = psi;
    reftraj(i,4) = v;
    reftraj(i,5) = cur;
    reftraj(i,6) = t;
    reftraj(i,7) = a;

end

reftraj(1,5) = reftraj(2,5);
reftraj(1,7) = reftraj(2,7);
path = reftraj(:,1:2);

save reftraj reftraj; 
