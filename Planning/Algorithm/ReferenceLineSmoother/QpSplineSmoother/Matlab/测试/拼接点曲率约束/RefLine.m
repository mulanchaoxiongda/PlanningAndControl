close all;
clear all;
clc;

x = [];
y = [];
theta = [];

radius = 6 / pi;
center = [0, radius];

for angle = pi/6 : pi/400 : pi/2
   x = [x, center(1) + radius * sin(angle)];
   y = [y, center(2) - radius * cos(angle)];
   theta = [theta, angle];
end

center = [radius * 2, radius];
radius = 6 / pi;

for angle = pi/400 : pi/400 : pi
   x = [x, center(1) - radius * cos(angle)];
   y = [y, center(2) + radius * sin(angle)];
   theta = [theta, -angle + pi / 2];
end

ref_path(:, 1) = x';
ref_path(:, 2) = y';
ref_path(:, 3) = theta';

save ref_path.mat ref_path