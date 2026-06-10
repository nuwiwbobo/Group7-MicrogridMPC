% run_simulation.m
% Simulasi Economic MPC lup-tertutup untuk microgrid selama 24 jam.
%
% Tujuan:
%   Menjalankan simulasi EMPC horizon-berjalan. Pada setiap langkah, SoC
%   saat ini diukur dan ugrid, ubatt optimal dihitung sepanjang horizon
%   prediksi. Hanya gerakan kontrol pertama yang diterapkan; sistem
%   berevolusi melalui model plant LTI.
%
% Konvensi tanda:
%   ubatt > 0 -> discharging (baterai mengirim daya)
%   ugrid > 0 -> impor dari jaringan
%   Ppv >= 0, Pload >= 0

%% Inisialisasi
clear; clc; close all;

params = parameters();
[A, B, C, D_ss, E, Fe] = plant_model(params);
Np = params.Np;
dT = params.dT;
Enom = params.Enom;

fprintf('\n=== Membangun matriks prediksi (sekali) ===\n');
[Phi_x, Gamma_x, Psi_x] = build_prediction_matrices(Np, dT, Enom);

fprintf('\n=== Membangkitkan prakiraan 24 jam ===\n');
D_forecast = generate_forecasts(params);

%% Simulasi lup-tertutup
Nsim = 24;
x_history = zeros(Nsim + 1, 1);
u_history = zeros(Nsim, 2);
cost_history = zeros(Nsim, 1);

x_history(1) = params.x0;

fprintf('\n=== Memulai simulasi lup-tertutup ===\n');
for k = 1:Nsim
    x_k = x_history(k);

    % Bangun prakiraan horizon (tangani akhir horizon)
    end_idx = min(k + Np - 1, 24);
    D_horizon = D_forecast(k:end_idx, :);
    if size(D_horizon, 1) < Np
        % Replikasi nilai terakhir (hindari infeasibility dari
        % Ppv=0, Pload=0 yang memaksa ugrid=-ubatt).
        last = D_horizon(end, :);
        pad = repmat(last, Np - size(D_horizon, 1), 1);
        D_horizon = [D_horizon; pad];
    end

    % Bangun dan selesaikan QP
    [H, f] = build_qp(x_k, D_horizon, params, Phi_x, Gamma_x);
    [A_eq, b_eq, A_ineq, b_ineq, lb, ub] = ...
        build_constraints(Np, params, x_k, D_horizon, Phi_x, Gamma_x);

    options = optimoptions('quadprog', 'Display', 'off');
    U_star = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, lb, ub, [], options);

    if isempty(U_star)
        error('QP tak-layak pada langkah %d. Periksa kendala.', k);
    end

    % Terapkan gerakan kontrol pertama
    u_k = U_star(1:2);
    u_history(k, :) = u_k';

    % Perbarui keadaan melalui model plant
    d_k = D_forecast(k, :)';
    x_next = A * x_k + B * u_k + E * d_k;
    x_next = max(params.SoCmin, min(params.SoCmax, x_next));
    x_history(k + 1) = x_next;

    % Rekam kontribusi biaya
    cost_history(k) = D_forecast(k, 3) * u_k(1) * dT;

    fprintf('  Langkah %2d: ugrid = %+6.2f kW, ubatt = %+6.2f kW, SoC = %.3f\n', ...
        k, u_k(1), u_k(2), x_history(k + 1));
end

fprintf('\nSimulasi selesai.\n');

%% Plot
hours = (0:Nsim - 1)';

figure('Position', [100, 100, 1200, 800]);

subplot(2, 3, 1);
plot(hours, D_forecast(:, 3), 'b-o', 'LineWidth', 1.5);
xlabel('Waktu [jam]'); ylabel('Harga [$/kWh]');
title('Harga Listrik'); grid on;

subplot(2, 3, 2);
plot(hours, u_history(:, 2), 'r-o', 'LineWidth', 1.5);
xlabel('Waktu [jam]'); ylabel('Daya [kW]');
title('Daya Baterai u_{batt}'); grid on;

subplot(2, 3, 3);
stairs(0:Nsim, x_history, 'k-', 'LineWidth', 1.5);
hold on; grid on;
yline(params.SoCmin, 'r--', 'SoC_{min}', 'LineWidth', 1.2);
yline(params.SoCmax, 'r--', 'SoC_{max}', 'LineWidth', 1.2);
xlabel('Waktu [jam]'); ylabel('SoC [-]');
title('State of Charge'); legend('SoC', 'SoC_{min}', 'SoC_{max}');

subplot(2, 3, 4);
cumulative_cost = cumsum(cost_history);
plot(hours, cumulative_cost, 'm-o', 'LineWidth', 1.5);
xlabel('Waktu [jam]'); ylabel('Biaya [$]');
title('Biaya Kumulatif'); grid on;

subplot(2, 3, 5);
plot(hours, u_history(:, 1), 'b-o', 'LineWidth', 1.5);
xlabel('Waktu [jam]'); ylabel('Daya [kW]');
title('Impor Jaringan u_{grid}'); grid on;

subplot(2, 3, 6);
plot(hours, D_forecast(:, 1), 'g-o', 'LineWidth', 1.5); hold on;
plot(hours, D_forecast(:, 2), 'm-s', 'LineWidth', 1.5);
xlabel('Waktu [jam]'); ylabel('Daya [kW]');
title('Prakiraan PV dan Beban'); legend('P_{pv}', 'P_{load}'); grid on;

sgtitle('Microgrid Economic MPC - Simulasi 24 Jam');
