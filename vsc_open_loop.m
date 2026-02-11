clear; clc; close all;
reset(groot);
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultFigureColor', 'w');
set(groot, 'defaultAxesBox', 'off');

m = 300;
b = 1500;
k = 20000;

A = [0, 1; -k/m, -b/m];

t_end = 5;
dt = 0.001;
t_span = 0:dt:t_end;

x0 = [-0.5; 0];

[t_out, x] = ode45(@(t, x) A * x, t_span, x0);

z = x(:, 1);
z_dot = x(:, 2);

figure;

subplot(2, 1, 1);
plot(t_out, z, 'r', 'LineWidth', 1.5);
xticks(0:0.1:t_end);
xlabel('Time (s)');
ylabel('Displacement (m)');
% title('Suspension Displacement');
grid on;
box off;

subplot(2, 1, 2);
plot(t_out, z_dot, 'g', 'LineWidth', 1.5);
xticks(0:0.1:t_end);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
% title('Suspension Velocity');
grid on;
box off;

sgtitle('Quarter-Car Suspension Free Response ($z_0 = -0.5$ m)', 'Interpreter', 'latex');
