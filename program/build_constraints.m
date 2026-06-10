function [A_eq, b_eq, A_ineq, b_ineq, lb, ub] = ...
    build_constraints(Np, params, x_k, D_horizon, Phi_x, Gamma_x)
%BUILD_CONSTRAINTS Membangun semua matriks kendala QP untuk EMPC.
%
% Tujuan:
%   Membangun kendala kesamaan, ketidaksamaan, dan batas untuk
%   program kuadratik Economic MPC microgrid.
%
% Masukan:
%   Np        - horizon prediksi [langkah]
%   params    - struct dengan field: Pbatt_max, Pgrid_max, SoCmin, SoCmax
%   x_k       - keadaan saat ini (SoC) [-]
%   D_horizon - matriks Np x 3 [Ppv, Pload, c] untuk horizon
%   Phi_x     - matriks prediksi Np x 1
%   Gamma_x   - matriks prediksi Np x (2*Np)
%
% Keluaran:
%   A_eq, b_eq - matriks kendala kesamaan (keseimbangan daya)
%   A_ineq, b_ineq - matriks kendala ketidaksamaan (batas SoC)
%   lb, ub     - batas bawah dan atas pada variabel keputusan
%
% Variabel keputusan: U = [ugrid(0); ubatt(0); ...; ugrid(Np-1); ubatt(Np-1)] (2Np x 1)
%
% Konvensi tanda:
%   ubatt > 0 -> discharging, ubatt < 0 -> charging
%   ugrid > 0 -> impor, ugrid < 0 -> ekspor

Pbatt_max = params.Pbatt_max;
Pgrid_max = params.Pgrid_max;
SoCmin    = params.SoCmin;
SoCmax    = params.SoCmax;

%% Ekstrak kolom prakiraan
Ppv_h   = D_horizon(:, 1);
Pload_h = D_horizon(:, 2);

%% Kesamaan: keseimbangan daya untuk setiap k
% Pload = Ppv + ubatt + ugrid  =>  ugrid + ubatt = Pload - Ppv
A_eq = zeros(Np, 2 * Np);
b_eq = zeros(Np, 1);
for k = 1:Np
    col = (k - 1) * 2 + 1;
    A_eq(k, col)     = 1;   % ugrid(k)
    A_eq(k, col + 1) = 1;   % ubatt(k)
    b_eq(k) = Pload_h(k) - Ppv_h(k);
end

%% Ketidaksamaan: batas SoC
% X = Phi_x * x_k + Gamma_x * U
% Bawah: X >= SoCmin  ->  -Gamma_x * U <= -SoCmin + Phi_x * x_k
% Atas:  X <= SoCmax  ->   Gamma_x * U <= SoCmax - Phi_x * x_k
A_ineq = [-Gamma_x; Gamma_x];
b_ineq = [-SoCmin * ones(Np, 1) + Phi_x * x_k; ...
           SoCmax * ones(Np, 1) - Phi_x * x_k];

%% Batas kotak pada ugrid dan ubatt
% Berselang-seling: [ugrid(0); ubatt(0); ugrid(1); ubatt(1); ...]
lb = zeros(2 * Np, 1);
ub = zeros(2 * Np, 1);
for k = 1:Np
    idx = (k - 1) * 2 + 1;
    % ugrid(k): -Pgrid_max <= ugrid <= Pgrid_max
    % Negatif = ekspor (net metering, diperlukan saat puncak surya)
    lb(idx)     = -Pgrid_max;
    ub(idx)     = Pgrid_max;
    % ubatt(k): -Pbatt_max <= ubatt <= Pbatt_max
    lb(idx + 1) = -Pbatt_max;
    ub(idx + 1) =  Pbatt_max;
end

end
