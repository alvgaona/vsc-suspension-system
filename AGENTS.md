# AGENTS.md - VSC (Variable Structure Control) Simulation

## Project Overview

MATLAB simulation of Variable Structure Control (sliding mode control) for different systems.
Self-contained scripts with no external dependencies, build system, or package manager.

### Files

| File | Description |
|---|---|
| `vsc.m` | Open-loop free response of the quarter-car suspension (no controller) |
| `vsc_controller.m` | VSC sliding mode controller with `sign(s*x1)` switching law |
| `vsc_controller_band.m` | VSC controller with boundary band for chattering reduction |
| `motorized_valve_example.m` | VSC controller for motorized valve (professor's PDF formulation) |

## Running Scripts

There is no build step. Run scripts directly in MATLAB:

```matlab
% Run from MATLAB command window or editor
run('vsc.m')
run('vsc_controller.m')
run('vsc_controller_band.m')
run('motorized_valve_example.m')
```

Or from the system shell:

```bash
matlab -nodisplay -nosplash -r "run('vsc.m'); exit"
matlab -nodisplay -nosplash -r "run('vsc_controller.m'); exit"
matlab -nodisplay -nosplash -r "run('vsc_controller_band.m'); exit"
matlab -nodisplay -nosplash -r "run('motorized_valve_example.m'); exit"
```

## Testing

There is no formal test framework. Validation is done by:
1. Running scripts and inspecting generated plots visually
2. Checking `fprintf` output for final state values (displacement, velocity, sliding surface)
3. Verifying phase portrait convergence to the origin

## Code Style Guidelines

### File Structure

Every script follows this structure in order:
1. Preamble: `clear; clc; close all;`
2. System parameters block
3. Controller parameters block (if applicable)
4. Simulation parameters block
5. ODE solver call (`ode45`)
6. Post-processing / signal computation loop
7. Visualization (figures, subplots, phase portraits)
8. Console output (`fprintf`)
9. Local function definitions (at end of file)

### File Naming

- Use **snake_case** for all `.m` filenames: `vsc_controller.m`, `vsc_controller_band.m`
- Names build hierarchically from base to specific variant

### Variable Naming

- **snake_case** for multi-word variables: `t_span`, `t_out`, `z_dot`, `x1_range`, `u_ctrl`
- **Single lowercase letters** for standard physics/math quantities: `m`, `b`, `k`, `t`, `x`, `s`
- **Uppercase single letters** for gains and forces following physics convention: `K`, `F`
- **Greek variable names** spelled out in full: `lambda`, `phi`
- **Derivative variables** use underscore notation: `dx1dt`, `dx2dt`, `dxdt`

### Function Naming and Style

- Local functions use **snake_case**: `vsc_dynamics`, `vsc_band_dynamics`
- Local functions are always defined **at the end of the file**
- Use tilde `~` for unused parameters (e.g., unused time argument in autonomous systems):
  ```matlab
  function dxdt = vsc_dynamics(~, x, m, b, k, lambda, K)
  ```
- For simple systems without local functions, anonymous functions are acceptable:
  ```matlab
  @(t, x) A * x
  ```
- Pass all parameters explicitly to functions; do not use global variables

### Comments

- **File-level header**: `%` block describing the system model and state-space formulation
  ```matlab
  % VSC Controller for Quarter-Car Suspension Model
  % System: m*z'' + b*z' + k*z = F
  % State-space: x1 = z (displacement), x2 = z_dot (velocity)
  ```
- **Section headers**: single `%` line with descriptive label (`% System parameters`, `% Visualization`)
- **Inline comments** for parameter units in square brackets: `m = 300; % mass [kg]`
- **Commented-out alternatives** are kept inline to document design options:
  ```matlab
  % u = (sx1 / phi) * K * x1;  % linear
  % u = K * tanh(sx1 / phi) * x1;  % sigmoid
  u = K * sin(pi/2 * sx1 / phi) * x1;  % sine
  ```
- Align inline parameter comments for readability

### Formatting

- Spaces around binary operators: `lambda * x1 + x2`, `(F - b*x2 - k*x1) / m`
- Spaces after commas in function calls and array literals: `zeros(n, 1)`, `plot(t, x(:,1))`
- Semicolons on all assignment lines to suppress MATLAB output
- Blank lines to separate logical sections
- No trailing whitespace

### ODE Solver Conventions

- Use `ode45` for all simulations
- Controller scripts must set explicit tolerances: `odeset('RelTol', 1e-6, 'AbsTol', 1e-9)`
- Time span as `[t_start t_end]` or `start:dt:end` vector
- Initial conditions as column vectors: `x0 = [-0.5; 0]`

### Plotting Conventions

- Line width of `1.5` for all data plots
- Single-character color codes: `'b'`, `'r'`, `'g'`, `'c'`, `'m'`
- Every subplot must include: `xlabel`, `ylabel`, `title`, `grid on`
- Zero reference lines: `yline(0, 'k--')`
- Overall figure title via `sgtitle`
- Phase portraits: green filled circle for initial point, red filled circle for final point
- Sliding surface shown as black dashed line (`'k--'`) on phase portraits

### Error Handling

- No explicit error handling is used in this codebase (academic simulation scripts)
- Rely on MATLAB's built-in error reporting for invalid operations

## Physics Model Reference

Quarter-car suspension: `m*z'' + b*z' + k*z = F`

State-space form:
- `x1 = z` (displacement)
- `x2 = z'` (velocity)
- `dx1/dt = x2`
- `dx2/dt = (F - b*x2 - k*x1) / m`

Default parameters: `m = 300 kg`, `b = 1500 Ns/m`, `k = 20000 N/m`

Sliding surface: `s = lambda * x1 + x2`

Switching law: `u = K * x1 * sign(s * x1)` (basic), with boundary band interpolation (band variant)
