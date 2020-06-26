close all;
clear all
clc;

fig1 = figure(1);
ax1 = axes('Parent', fig1, 'xlim',  [0, 15]);
axis equal
hold(ax1, 'on')


plane = [-1, 1, 0, 3, 0, 0, 1.5]';
sphere = [10, 5, 0, 0.5]';

x = 1:0.5:10;
y = (dot(plane(1:3), plane(4:6)) - plane(1) * x)/plane(2);

plot(ax1, x, y);
plot(ax1, plane(4), plane(5), 'ro');



rectangle('Parent', ax1, 'Position', [sphere(1) - sphere(4) sphere(2) - sphere(4) 2 * sphere(4) 2 * sphere(4)], 'Curvature', 1);

a = planeCollisionAvoidance(plane, sphere);
disp("if greater than zero then no collision");
disp(a);
