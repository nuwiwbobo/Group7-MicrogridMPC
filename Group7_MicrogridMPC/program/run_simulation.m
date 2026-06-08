% run_simulation.m
% Closed-loop Economic MPC simulation for the microgrid over 24 hours.
%
% Purpose:
%   Runs a receding-horizon EMPC simulation. At each step, the current
%   SoC is measured and the optimal ugrid, ubatt are computed over the
%   prediction horizon. Only the first control move is applied; the
%   system evolves via the LTI plant model.
%
% Sign convention:
%   ubatt > 0 -> discharging (battery delivers power)
%   ugrid > 0 -> importing from grid
%   Ppv >= 0, Pload >= 0

%% Initialisation
clear; clc; close all;

params = parameters();
[A, B, C, D_ss, E, Fe] = plant_model(params);
Np = params.Np;
dT = params.dT;
Enom = params.Enom;

fprintf('\n=== Building prediction matrices (once) ===\n');
[Phi_x, Gamma_x, Psi_x] = build_prediction_matrices(Np, dT, Enom);

fprintf('\n=== Generating 24-hour forecasts ===\n');
D_forecast = generate_forecasts(params);

%% Closed-loop simulation
Nsim = 24;
x_history = zeros(Nsim + 1, 1);
u_history = zeros(Nsim, 2);
cost_history = zeros(Nsim, 1);

x_history(1) = params.x0;

fprintf('\n=== Starting closed-loop simulation ===\n');
for k = 1:Nsim
    x_k = x_history(k);

    % Build horizon forecast (handle end of horizon)
    end_idx = min(k + Np - 1, 24);
    D_horizon = D_forecast(k:end_idx, :);
    if size(D_horizon, 1) < Np
        % Zero-pad if horizon extends beyond available forecast
        pad = zeros(Np - size(D_horizon, 1), 3);
        D_horizon = [D_horizon; pad];
    end

    % Build and solve QP
    [H, f] = build_qp(x_k, D_horizon, params, Phi_x, Gamma_x);
    [A_eq, b_eq, A_ineq, b_ineq, lb, ub] = ...
        build_constraints(Np, params, x_k, D_horizon, Phi_x, Gamma_x);

    options = optimoptions('quadprog', 'Display', 'off');
    U_star = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, lb, ub, [], options);

    % Apply first control move
    u_k = U_star(1:2);
    u_history(k, :) = u_k';

    % Update state via plant model
    d_k = D_forecast(k, :)';
    x_next = A * x_k + B * u_k + E * d_k;
    x_next = max(params.SoCmin, min(params.SoCmax, x_next));
    x_history(k + 1) = x_next;

    % Record cost contribution
    cost_history(k) = D_forecast(k, 3) * u_k(1) * dT;

    fprintf('  Step %2d: ugrid = %+6.2f kW, ubatt = %+6.2f kW, SoC = %.3f\n', ...
        k, u_k(1), u_k(2), x_history(k + 1));
end

fprintf('\nSimulation complete.\n');

%% Plotting
hours = (0:Nsim - 1)';

figure('Position', [100, 100, 1200, 800]);

subplot(2, 3, 1);
plot(hours, D_forecast(:, 3), 'b-o', 'LineWidth', 1.5);
xlabel('Time [h]'); ylabel('Price [$/kWh]');
title('Electricity Price'); grid on;

subplot(2, 3, 2);
plot(hours, u_history(:, 2), 'r-o', 'LineWidth', 1.5);
xlabel('Time [h]'); ylabel('Power [kW]');
title('Battery Power u_{batt}'); grid on;

subplot(2, 3, 3);
stairs(0:Nsim, x_history, 'k-', 'LineWidth', 1.5);
hold on; grid on;
yline(params.SoCmin, 'r--', 'SoC_{min}', 'LineWidth', 1.2);
yline(params.SoCmax, 'r--', 'SoC_{max}', 'LineWidth', 1.2);
xlabel('Time [h]'); ylabel('SoC [-]');
title('State of Charge'); legend('SoC', 'SoC_{min}', 'SoC_{max}');

subplot(2, 3, 4);
cumulative_cost = cumsum(cost_history);
plot(hours, cumulative_cost, 'm-o', 'LineWidth', 1.5);
xlabel('Time [h]'); ylabel('Cost [$]');
title('Cumulative Cost'); grid on;

subplot(2, 3, 5);
plot(hours, u_history(:, 1), 'b-o', 'LineWidth', 1.5);
xlabel('Time [h]'); ylabel('Power [kW]');
title('Grid Import u_{grid}'); grid on;

subplot(2, 3, 6);
plot(hours, D_forecast(:, 1), 'g-o', 'LineWidth', 1.5); hold on;
plot(hours, D_forecast(:, 2), 'm-s', 'LineWidth', 1.5);
xlabel('Time [h]'); ylabel('Power [kW]');
title('PV and Load Forecast'); legend('P_{pv}', 'P_{load}'); grid on;

sgtitle('Microgrid Economic MPC - 24h Simulation');
