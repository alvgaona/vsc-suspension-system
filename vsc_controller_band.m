% VSC Controller with Boundary Band
% System: m*z'' + b*z' + k*z = u
% State-space: x1 = z (displacement), x2 = z_dot (velocity)

clear; clc; close all;

reset(groot);
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultFigureColor', 'w');
set(groot, 'defaultAxesBox', 'off');

% System parameters
m = 300;        % mass [kg]
b = 1500;       % damping coefficient [Ns/m]
k = 20000;      % spring stiffness [N/m]

% VSC parameters
lambda = 0.5;   % sliding surface coefficient
K = abs(-lambda*lambda*m - k + b*lambda - 1);    % control gain
phi = 0.05;     % boundary band threshold
band_mode = 'linear'; % 'linear', 'sine', 'tanh', 'sigmoid'
N = (k + K);   % precompensation gain for zero steady-state error
% Reference signal: step sequence
r_values = [0];   % step amplitudes [m]
r_times  = [0];   % step times [s]

% Simulation parameters
tspan = [0 10];
x0 = [-0.5; 0]; % initial conditions at rest

% Run simulation
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t, x] = ode45(@(t, x) vsc_band_dynamics(t, x, m, b, k, lambda, K, phi, N, r_values, r_times, band_mode), tspan, x0, options);

% Compute control signals for plotting
n = length(t);
s = zeros(n, 1);
u_ctrl = zeros(n, 1);
r_sig = zeros(n, 1);

for i = 1:n
    x1 = x(i,1);
    x2 = x(i,2);
    s(i) = lambda * x1 + x2;
    sx1 = s(i) * x1;

    r = r_values(find(r_times <= t(i), 1, 'last'));
    r_sig(i) = r;

    if sx1 >= phi
        psi = K;
    elseif sx1 <= -phi
        psi = -K;
    else
        xi = sx1 / phi;
        switch band_mode
            case 'linear',  psi = K * xi;
            case 'sine',    psi = K * sin(pi/2 * xi);
            case 'tanh',    psi = K * tanh(xi);
            case 'sigmoid', psi = K * (2 / (1 + exp(-xi)) - 1);
        end
    end
    u_ctrl(i) = N * r - psi * x1;
end

% Visualization
figure('Position', [100 100 900 1000]);

subplot(4,2,1);
plot(t, x(:,1), 'b', 'LineWidth', 1.5);
hold on;
plot(t, r_sig, 'k--', 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Displacement $z$ [m]');
title('Displacement $z(t)$');
legend('$z$', '$r(t)$', 'Location', 'best');
grid on; box off;

subplot(4,2,2);
plot(t, x(:,2), 'r', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Velocity $\dot{z}$ [m/s]');
title('Velocity $\dot{z}(t)$');
grid on; box off;
yline(0, 'k--');

subplot(4,2,3);
plot(t, s, 'm', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('$s$');
grid on; box off;
yline(0, 'k--');

subplot(4,2,4);
sx1_plot = s .* x(:,1);
plot(t, sign(sx1_plot), 'g', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('$\mathrm{sign}(s \cdot x_1)$');
title('Switching Condition $\mathrm{sign}(s \cdot x_1)$');
grid on; box off;
ylim([-1.5 1.5]);
yline(0, 'k--');

subplot(4,2,5);
plot(t, u_ctrl/1000, 'c', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Control Input $u$ [kN]');
title('Control Input $u(t)$');
grid on; box off;
yline(0, 'k--');

subplot(4,2,6);
hold on;
sx1_range = linspace(-3*phi, 3*phi, 500);
psi_band = zeros(size(sx1_range));
for i = 1:length(sx1_range)
    if sx1_range(i) >= phi
        psi_band(i) = K;
    elseif sx1_range(i) <= -phi
        psi_band(i) = -K;
    else
        xi = sx1_range(i) / phi;
        switch band_mode
            case 'linear',  psi_band(i) = K * xi;
            case 'sine',    psi_band(i) = K * sin(pi/2 * xi);
            case 'tanh',    psi_band(i) = K * tanh(xi);
            case 'sigmoid', psi_band(i) = K * (2 / (1 + exp(-xi)) - 1);
        end
    end
end
h1 = plot([-3*phi, 0], [-K, -K], 'b', 'LineWidth', 1.5);
plot([0, 3*phi], [K, K], 'b', 'LineWidth', 1.5);
h2 = plot(sx1_range, psi_band, 'r', 'LineWidth', 1.5);
xline(0, 'k--');
xline(-phi, 'k--');
xline(phi, 'k--');
xlabel('$s \cdot x_1$');
ylabel('$\psi$');
legend([h1, h2], '$\phi = 0$', '$\phi > 0$', 'Location', 'best');
ylim([-1.5*K, 1.5*K]);
grid on; box off;

subplot(4,2,[7 8]);
hold on;
x1_min = min(x(:,1)) - 0.1;
x1_max = max(x(:,1)) + 0.1;
x1_range = linspace(x1_min, x1_max, 100);
x2_upper = -lambda * x1_range + phi;
x2_lower = -lambda * x1_range - phi;
fill([x1_range fliplr(x1_range)], [x2_upper fliplr(x2_lower)], 'y', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(x1_range, x2_upper, 'r--', 'LineWidth', 0.5);
plot(x1_range, x2_lower, 'r--', 'LineWidth', 0.5);
plot(x1_range, -lambda * x1_range, 'k--', 'LineWidth', 1);
plot(x(:,1), x(:,2), 'b', 'LineWidth', 1.5);
plot(x(1,1), x(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(x(end,1), x(end,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('$x_1$');
ylabel('$x_2$');
% title('Phase Portrait');
grid on; box off;

sgtitle('VSC Controller with Boundary Band', 'Interpreter', 'latex');

% Analytical metrics (linear equivalent: Structure I with psi = K)
% Closed-loop: z'' + (b/m)*z' + (k+K)/m * z = N*r/m
wn = sqrt((k + K) / m);
zeta = b / (2 * m * wn);
wd = wn * sqrt(1 - zeta^2);
tr_a = (pi - acos(zeta)) / wd;
tp_a = pi / wd;
Mp_a = exp(-zeta * pi / sqrt(1 - zeta^2)) * 100;
ts_a = 4 / (zeta * wn);

fprintf('\n--- Analytical Metrics (Linear Equivalent) ---\n');
fprintf('  wn   = %.4f rad/s\n', wn);
fprintf('  zeta = %.4f\n', zeta);
fprintf('  tr   = %.4f s\n', tr_a);
fprintf('  tp   = %.4f s\n', tp_a);
fprintf('  Mp   = %.2f %%\n', Mp_a);
fprintf('  ts   = %.4f s\n', ts_a);

% Numerical step response metrics
x1_init = x0(1);
x1_ss = r_values(1);
delta = x1_ss - x1_init;

lvl_10 = x1_init + 0.1 * delta;
lvl_90 = x1_init + 0.9 * delta;
if delta > 0
    idx_10 = find(x(:,1) >= lvl_10, 1, 'first');
    idx_90 = find(x(:,1) >= lvl_90, 1, 'first');
else
    idx_10 = find(x(:,1) <= lvl_10, 1, 'first');
    idx_90 = find(x(:,1) <= lvl_90, 1, 'first');
end
if ~isempty(idx_10) && ~isempty(idx_90)
    tr_n = t(idx_90) - t(idx_10);
else
    tr_n = NaN;
end

if delta > 0
    [x1_peak, idx_peak] = max(x(:,1));
else
    [x1_peak, idx_peak] = min(x(:,1));
end
tp_n = t(idx_peak);
Mp_n = abs(x1_peak - x1_ss) / abs(delta) * 100;

tol = 0.02 * abs(delta);
settled = abs(x(:,1) - x1_ss) <= tol;
idx_last_outside = find(~settled, 1, 'last');
if isempty(idx_last_outside)
    ts_n = 0;
elseif idx_last_outside < length(t)
    ts_n = t(idx_last_outside + 1);
else
    ts_n = NaN;
end

fprintf('\n--- Numerical Metrics ---\n');
fprintf('  tr = %.4f s\n', tr_n);
fprintf('  tp = %.4f s\n', tp_n);
fprintf('  Mp = %.2f %%\n', Mp_n);
fprintf('  ts = %.4f s\n', ts_n);

fprintf('\nFinal displacement: %.6f m\n', x(end,1));
fprintf('Final velocity: %.6f m/s\n', x(end,2));
fprintf('Final sliding surface: %.6f\n', s(end));

function dxdt = vsc_band_dynamics(t, x, m, b, k, lambda, K, phi, N, r_values, r_times, band_mode)
    x1 = x(1);
    x2 = x(2);

    r = r_values(find(r_times <= t, 1, 'last'));

    s = lambda * x1 + x2;
    sx1 = s * x1;

    if sx1 >= phi
        psi = K;
    elseif sx1 <= -phi
        psi = -K;
    else
        xi = sx1 / phi;
        switch band_mode
            case 'linear',  psi = K * xi;
            case 'sine',    psi = K * sin(pi/2 * xi);

            case 'tanh',    psi = K * tanh(xi);
            case 'sigmoid', psi = K * (2 / (1 + exp(-xi)) - 1);
        end
    end
    u = N * r - psi * x1;

    dx1dt = x2;
    dx2dt = (u - b*x2 - k*x1) / m;

    dxdt = [dx1dt; dx2dt];
end
