% VSC Controller for Motorized Valve Flow Regulation
% System: d*x'' + b*x' + a*x = u
% State-space: x1 = x (valve position), x2 = x' (valve velocity)

clear; clc; close all;

% System parameters
a = 3.912;
b = 4.186;
d = 5.086;

% VSC parameters
c = 0.5;   % sliding surface coefficient
K = 0.5;   % control gain (psi = alpha = K when sx1 > 0, psi = beta = -K when sx1 < 0)
phi = 0.1; % boundary band threshold
% N = (a + d * K) / d; % precompensation gain for zero steady-state error
N = 1;

% Reference signal: step sequence
r_values = [1, 2, 0.5];   % step amplitudes
r_times  = [0, 20, 40];   % step times [s]

% Simulation parameters
tspan = [0 60];
x0 = [0; 0]; % initial conditions at rest

% Run simulation
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t, x] = ode45(@(t, x) vsc_band_dynamics(t, x, a, b, d, c, K, phi, N, r_values, r_times), tspan, x0, options);

% Compute control signals for plotting
n = length(t);
s = zeros(n, 1);
u_ctrl = zeros(n, 1);
psi_sig = zeros(n, 1);
r_sig = zeros(n, 1);

for i = 1:n
    x1 = x(i,1);
    x2 = x(i,2);
    s(i) = c * x1 + x2;
    sx1 = s(i) * x1;

    r = r_values(find(r_times <= t(i), 1, 'last'));
    r_sig(i) = r;

    if sx1 >= phi
        psi = K;
    elseif sx1 <= -phi
        psi = -K;
    else
        psi = (sx1 / phi) * K;  % linear
        % psi = K * tanh(sx1 / phi);  % sigmoid
        % psi = K * sin(pi/2 * sx1 / phi);  % sine
    end
    psi_sig(i) = psi;
    u_ctrl(i) = N * r - psi * x1;
end

% Analytical metrics (linear equivalent: Structure I with psi = K)
% Closed-loop: x'' + (b/d)*x' + (a + d*K)/d * x = N*r
wn = sqrt((a + d * K) / d);
zeta = (b / d) / (2 * wn);
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

% Numerical step response metrics for each step
fprintf('\n--- Numerical Step Response Metrics ---\n');
for j = 1:length(r_values)
    t_start = r_times(j);
    if j < length(r_values)
        t_end = r_times(j + 1);
    else
        t_end = tspan(2);
    end

    idx = (t >= t_start) & (t < t_end);
    t_seg = t(idx) - t_start;
    x1_seg = x(idx, 1);

    x1_init = x1_seg(1);
    x1_ss = r_values(j);
    delta = x1_ss - x1_init;

    % Rise time (10% to 90%)
    lvl_10 = x1_init + 0.1 * delta;
    lvl_90 = x1_init + 0.9 * delta;
    if delta > 0
        idx_10 = find(x1_seg >= lvl_10, 1, 'first');
        idx_90 = find(x1_seg >= lvl_90, 1, 'first');
    else
        idx_10 = find(x1_seg <= lvl_10, 1, 'first');
        idx_90 = find(x1_seg <= lvl_90, 1, 'first');
    end
    if ~isempty(idx_10) && ~isempty(idx_90)
        tr = t_seg(idx_90) - t_seg(idx_10);
    else
        tr = NaN;
    end

    % Peak time and maximum overshoot
    if delta > 0
        [x1_peak, idx_peak] = max(x1_seg);
    else
        [x1_peak, idx_peak] = min(x1_seg);
    end
    tp = t_seg(idx_peak);
    Mp = abs(x1_peak - x1_ss) / abs(delta) * 100;

    % Settling time (2% band)
    tol = 0.02 * abs(delta);
    settled = abs(x1_seg - x1_ss) <= tol;
    idx_last_outside = find(~settled, 1, 'last');
    if isempty(idx_last_outside)
        ts = 0;
    elseif idx_last_outside < length(t_seg)
        ts = t_seg(idx_last_outside + 1);
    else
        ts = NaN;
    end

    fprintf('Step %d (r = %.1f, t = %.0f s):\n', j, r_values(j), t_start);
    fprintf('  tr = %.4f s\n', tr);
    fprintf('  tp = %.4f s\n', tp);
    fprintf('  Mp = %.2f %%\n', Mp);
    fprintf('  ts = %.4f s\n', ts);
end
fprintf('\n');

% Visualization
figure('Position', [100 100 900 800]);

subplot(3,2,1);
plot(t, x(:,1), 'b', 'LineWidth', 1.5);
hold on;
plot(t, r_sig, 'k--', 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Valve Position x_1');
title('Valve Position x_1(t)');
legend('x_1', 'r(t)', 'Location', 'best');
grid on;

subplot(3,2,2);
plot(t, x(:,2), 'r', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Valve Velocity x_2');
title('Valve Velocity x_2(t)');
grid on;
yline(0, 'k--');

subplot(3,2,3);
sx1_plot = s .* x(:,1);
plot(t, sign(sx1_plot), 'g', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('sign(s \cdot x_1)');
title('Switching Condition sign(s \cdot x_1)');
grid on;
ylim([-1.5 1.5]);
yline(0, 'k--');

subplot(3,2,4);
plot(t, u_ctrl, 'c', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Control Input u');
title('Control Input u(t)');
grid on;
yline(0, 'k--');

subplot(3,2,5);
sx1_range = linspace(-3*phi, 3*phi, 500);
psi_plot = zeros(size(sx1_range));
for i = 1:length(sx1_range)
    if sx1_range(i) >= phi
        psi_plot(i) = K;
    elseif sx1_range(i) <= -phi
        psi_plot(i) = -K;
    else
        psi_plot(i) = (sx1_range(i) / phi) * K;
    end
end
plot(sx1_range, psi_plot, 'm', 'LineWidth', 1.5);
xlabel('$s \cdot x_1$');
ylabel('$\psi$');
title('Control Gain $\psi(s \cdot x_1)$');
grid on;
yline(0, 'k--');
xline(-phi, 'k--');
xline(phi, 'k--');

subplot(3,2,6);
hold on;
x1_min = min(x(:,1)) - 0.1;
x1_max = max(x(:,1)) + 0.1;
x1_range = linspace(x1_min, x1_max, 100);
x2_upper = -c * x1_range + phi;
x2_lower = -c * x1_range - phi;
fill([x1_range fliplr(x1_range)], [x2_upper fliplr(x2_lower)], 'y', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(x1_range, x2_upper, 'r--', 'LineWidth', 0.5);
plot(x1_range, x2_lower, 'r--', 'LineWidth', 0.5);
plot(x1_range, -c * x1_range, 'k--', 'LineWidth', 1);
plot(x(:,1), x(:,2), 'b', 'LineWidth', 1.5);
plot(x(1,1), x(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(x(end,1), x(end,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('x_1');
ylabel('x_2');
title('Phase Portrait');
grid on;

sgtitle('VSC Controller - Motorized Valve Flow Regulation');

fprintf('Final valve position: %.6f\n', x(end,1));
fprintf('Final valve velocity: %.6f\n', x(end,2));
fprintf('Final sliding surface: %.6f\n', s(end));

function dxdt = vsc_band_dynamics(t, x, a, b, d, c, K, phi, N, r_values, r_times)
    x1 = x(1);
    x2 = x(2);

    r = r_values(find(r_times <= t, 1, 'last'));

    s = c * x1 + x2;
    sx1 = s * x1;

    if sx1 >= phi
        u = K * x1;
    elseif sx1 <= -phi
        u = -K * x1;
    else
        u = (sx1 / phi) * K * x1;  % linear
        % u = K * tanh(sx1 / phi) * x1;  % sigmoid
        % u = K * sin(pi/2 * sx1 / phi) * x1;  % sine
    end

    u = N * r - u;

    dx1dt = x2;
    dx2dt = -b*x2 - a*x1 + d*u;

    dxdt = [dx1dt; dx2dt];
end
