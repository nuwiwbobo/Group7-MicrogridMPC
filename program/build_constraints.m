function [A_eq, b_eq, A_ineq, b_ineq, lb, ub] = ...
    build_constraints(Np, params, x_k, D_horizon, Phi_x, Gamma_x)
%BUILD_CONSTRAINTS Builds all QP constraint matrices for the EMPC problem.
%
% Purpose:
%   Constructs equality, inequality, and bound constraints for the
%   microgrid Economic MPC quadratic program.
%
% Inputs:
%   Np        - prediction horizon [steps]
%   params    - struct with fields: Pbatt_max, Pgrid_max, SoCmin, SoCmax
%   x_k       - current state (SoC) [-]
%   D_horizon - Np x 3 matrix [Ppv, Pload, c] for the horizon
%   Phi_x     - Np x 1 prediction matrix
%   Gamma_x   - Np x (2*Np) prediction matrix
%
% Outputs:
%   A_eq, b_eq - equality constraint matrices (power balance)
%   A_ineq, b_ineq - inequality constraint matrices (SoC bounds)
%   lb, ub     - lower and upper bounds on decision variables
%
% Decision variable: U = [ugrid(0); ubatt(0); ...; ugrid(Np-1); ubatt(Np-1)] (2Np x 1)
%
% Sign convention:
%   ubatt > 0 -> discharging, ubatt < 0 -> charging
%   ugrid > 0 -> importing, ugrid < 0 -> exporting (lower bound = 0)

Pbatt_max = params.Pbatt_max;
Pgrid_max = params.Pgrid_max;
SoCmin    = params.SoCmin;
SoCmax    = params.SoCmax;

%% Extract forecast columns
Ppv_h   = D_horizon(:, 1);
Pload_h = D_horizon(:, 2);

%% Equality: power balance for each k
% ugrid(k) - ubatt(k) = Pload(k) - Ppv(k)
A_eq = zeros(Np, 2 * Np);
b_eq = zeros(Np, 1);
for k = 1:Np
    col = (k - 1) * 2 + 1;
    A_eq(k, col)     = 1;   % ugrid(k)
    A_eq(k, col + 1) = -1;  % ubatt(k)
    b_eq(k) = Pload_h(k) - Ppv_h(k);
end

%% Inequality: SoC bounds
% X = Phi_x * x_k + Gamma_x * U
% Lower: X >= SoCmin  ->  -Gamma_x * U <= -SoCmin + Phi_x * x_k
% Upper: X <= SoCmax  ->   Gamma_x * U <= SoCmax - Phi_x * x_k
A_ineq = [-Gamma_x; Gamma_x];
b_ineq = [-SoCmin * ones(Np, 1) + Phi_x * x_k; ...
           SoCmax * ones(Np, 1) - Phi_x * x_k];

%% Box bounds on ugrid and ubatt
% Interleaved: [ugrid(0); ubatt(0); ugrid(1); ubatt(1); ...]
lb = zeros(2 * Np, 1);
ub = zeros(2 * Np, 1);
for k = 1:Np
    idx = (k - 1) * 2 + 1;
    % ugrid(k): -Pgrid_max <= ugrid <= Pgrid_max
    % Negative = export (net metering, required for solar peak)
    lb(idx)     = -Pgrid_max;
    ub(idx)     = Pgrid_max;
    % ubatt(k): -Pbatt_max <= ubatt <= Pbatt_max
    lb(idx + 1) = -Pbatt_max;
    ub(idx + 1) =  Pbatt_max;
end

end
