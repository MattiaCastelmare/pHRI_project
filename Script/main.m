clear all
clc
close all

simulation_time = 30;

ris = sim('Simulation_traj.slx');
move_robot

% Quantities from simulation
F = ris.F;
Fx = ris.Fx;
Fy = ris.Fy;
Fz = ris.Fz;

cart = ris.cart;
residual = ris.residual;

x = ris.x;
y = ris.y;
z = ris.z;
xd = ris.xd;
yd = ris.yd;
zd = ris.zd;

dx_des = ris.dx_des;
dy_des = ris.dy_des;
dz_des = ris.dz_des;
dx = ris.dx;
dy = ris.dy;
dz = ris.dz;

taujx = ris.taujx;
taujy = ris.taujy;
taujz = ris.taujz;

limit = size(Fx);
time = linspace(0,30, limit(3)); 

% Enable LaTeX interpreter for legends
set(groot, 'defaultLegendInterpreter', 'latex');

% Set colors
lightRed = [1, 0.5, 0];
lightBlue = [0, 0.5, 1];
darkGreen = [0, 0.5, 0];

% Create the subplots
subplot(4, 1, 1); 
plot(time, squeeze(x), 'Color', lightRed, 'DisplayName','x', 'LineWidth', 2);
title('End effector position');
xlabel('time');

hold on

plot(time, squeeze(y), 'Color', lightBlue,'DisplayName','y', 'LineWidth', 2);
plot(time, squeeze(z), 'Color', darkGreen, 'DisplayName','z', 'LineWidth', 2);
ylim([-1, 1]); 
legend({'x', 'y', 'z'}, 'Location', 'best');

% Define interval start times and labels from 'A' to 'I'
interval_starts = [0, 3, 6, 12, 16, 18, 23, 26, 29];
interval_labels = char('A':'I'); % Letters from 'A' to 'I'

% Calculate the middle time for each interval
interval_midpoints = (interval_starts(1:end-1) + interval_starts(2:end)) / 2;

% Add interval labels on top of the plot
for i = 1:length(interval_midpoints)
    text(interval_midpoints(i), 1.35, interval_labels(i), 'FontSize', 12, 'HorizontalAlignment', 'center');
end

xline(3,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(6,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(12, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(16, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(18, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(23, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(26, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(29, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
grid on;

subplot(4, 1, 2);
plot(time, squeeze(dx), 'Color', lightRed, 'DisplayName','dx', 'LineWidth', 2);
title('End effector velocity');
xlabel('time');

hold on

plot(time, squeeze(dy), 'Color', lightBlue,'DisplayName','dy', 'LineWidth', 2);
plot(time, squeeze(dz), 'Color', darkGreen, 'DisplayName','dz', 'LineWidth', 2);
ylim([-0.25,0.25]); 
legend({'$\dot{x}$', '$\dot{y}$', '$\dot{z}$'}, 'Location', 'best');

xline(3,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(6,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(12, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(16, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(18, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(23, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(26, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(29, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
grid on;

subplot(4, 1, 3); 
plot(time, squeeze(Fx), 'Color', lightRed, 'DisplayName','Fx', 'LineWidth', 2);
title('Forces');
xlabel('time');
hold on
plot(time, squeeze(Fy), 'Color', lightBlue,'DisplayName','Fy', 'LineWidth', 2);
plot(time, squeeze(Fz), 'Color', darkGreen, 'DisplayName','Fz', 'LineWidth', 2);
ylim([-60,50]); 
legend({'Fx', 'Fy', 'Fz'}, 'Location', 'best');

xline(3,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(6,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(12, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(16, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(18, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(23, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(26, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(29, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
grid on;

subplot(4, 1, 4); 
plot(time, residual, 'r', 'DisplayName','Energy residual', 'LineWidth', 2);
title('Energy residual');
xlabel('time');
legend({'Energy residual'}, 'Location', 'best');
xline(3,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(6,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(12, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(16, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(18, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(23, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(26, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(29, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');

hold on
yline(2,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(-2, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
ylim([-5,10]);
grid on;
hold off;

% Create the subplots
figure;
subplot(2, 1, 1);
plot(time, squeeze(x), 'Color', lightRed, 'DisplayName','x', 'LineWidth', 2);
title('End effector position');
xlabel('time');

hold on

plot(time, squeeze(y), 'Color', lightBlue,'DisplayName','y', 'LineWidth', 2);
plot(time, squeeze(z), 'Color', darkGreen, 'DisplayName','z', 'LineWidth', 2);
plot(time, squeeze(xd), 'k--', 'DisplayName', '$x_{d}$', 'LineWidth', 1.5);
plot(time, squeeze(yd), 'k--', 'DisplayName', '$y_{d}$', 'LineWidth', 1.5);
plot(time, squeeze(zd), 'k--', 'DisplayName', '$z_{d}$', 'LineWidth', 1.5);
ylim([-1, 1]);
legend({'x', 'y', 'z'}, 'Location', 'best');

% Define interval start times and labels from 'A' to 'I'
interval_starts = [0, 3, 6, 12, 16, 18, 23, 26, 29];
interval_labels = char('A':'I'); % Letters from 'A' to 'I'

% Calculate the middle time for each interval
interval_midpoints = (interval_starts(1:end-1) + interval_starts(2:end)) / 2;

% Add interval labels on top of the plot
for i = 1:length(interval_midpoints)
    text(interval_midpoints(i), 1.35, interval_labels(i), 'FontSize', 12, 'HorizontalAlignment', 'center');
end

xline(3,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(6,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(12, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(16, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(18, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(23, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(26, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(29, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
grid on;

subplot(2, 1, 2);
plot(time, squeeze(dx), 'Color', lightRed, 'DisplayName','dx', 'LineWidth', 2);
title('End effector velocity');
xlabel('time');

hold on

plot(time, squeeze(dy), 'Color', lightBlue,'DisplayName','dy', 'LineWidth', 2);
plot(time, squeeze(dz), 'Color', darkGreen, 'DisplayName','dz', 'LineWidth', 2);
plot(time, squeeze(dx_des), 'k--', 'DisplayName', '$\dot{x}_{d}$', 'LineWidth', 1.5);
plot(time, squeeze(dy_des), 'k--', 'DisplayName', '$\dot{y}_{d}$', 'LineWidth', 1.5);
plot(time, squeeze(dz_des), 'k--', 'DisplayName', '$\dot{z}_{d}$', 'LineWidth', 1.5);
ylim([-0.25,0.25]);
legend({'$\dot{x}$', '$\dot{y}$', '$\dot{z}$'}, 'Location', 'best');

xline(3,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(6,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(12, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(16, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(18, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(23, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(26, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(29, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
grid on;

figure;

plot(time, residual, 'r', 'DisplayName','Energy residual', 'LineWidth', 2);
title('Energy residual');
xlabel('time');
legend({'Energy residual'}, 'Location', 'best');
xline(3,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(6,  'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(12, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(16, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(18, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(23, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(26, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(29, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
hold on
yline(2,  'k--', 'LineWidth', 2, 'HandleVisibility', 'off'); 
yline(-2, 'k--', 'LineWidth', 2, 'HandleVisibility', 'off'); 

ylim([-5,10]); 
grid on;
hold off;
