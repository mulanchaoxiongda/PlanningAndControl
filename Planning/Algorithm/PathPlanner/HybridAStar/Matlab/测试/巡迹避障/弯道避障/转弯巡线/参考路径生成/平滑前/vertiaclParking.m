close all;
clear all;
clc;

ref_x = [];
ref_y = [];
ref_theta = [];

for ds = 1.0 : 0.01 : 7.0
    ref_x = [ref_x, 1.0];
    ref_y = [ref_y, ds];
    ref_theta = [ref_theta, pi/2];
end

R = 2.0;
for theta = 0 : pi/180 : pi / 2
    ref_x = [ref_x, 1.0+R - R * cos(theta)];
    ref_y = [ref_y, 7.0 + R * sin(theta)];
    ref_theta = [ref_theta, theta];
end

for ds = 1.0 + R : 0.01 : 10.0
    ref_x = [ref_x, ds];
    ref_y = [ref_y, 7.0+R];
    ref_theta = [ref_theta, 0.0];
end

plot(ref_x, ref_y,'ro'); axis equal

turn_data(:, 1) = ref_x';
turn_data(:, 2) = ref_y';
turn_data(:, 3) = ref_theta';

save turn.mat turn_data;
