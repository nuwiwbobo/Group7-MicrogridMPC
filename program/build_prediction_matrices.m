function [Phi_x, Gamma_x, Psi_x] = build_prediction_matrices(p)
%BUILD_PREDICTION_MATRICES  Stacked MPC prediction matrices (F and Phi).
%   [Phi_x, Gamma_x, Psi_x] = BUILD_PREDICTION_MATRICES(p) constructs
%   the stacked prediction form X = Phi_x * x(k) + Gamma_x * U + Psi_x * D,
%   based on the plant model x(k+1) = x(k) + B*u(k) (A=1, E=0 in v1).
%
%   Input:
%       p  - parameter struct (p.E_nom, p.delta_T, p.Np)
%
%   Outputs:
%       Phi_x   - Np x 1    (state contribution — all ones)
%       Gamma_x - Np x 2Np  (control contribution)
%       Psi_x   - Np x 3Np  (disturbance contribution — zero in v1)
%
%   Reference:
%       Rawlings, Mayne, Diehl 2017, Ch. 2 (standard stacked MPC form).
%       The assignment calls these matrices F and Phi respectively.
%       Note: the design spec 2026-06-05 pseudocode had a factor
%       (i-j+1) in the Gamma_x loop — that is corrected here per
%       the standard derivation (A=1, so A^(i-j)*B = B, not (i-j+1)*B).
%
% Group 7 - Microgrid Economic MPC
% Sistem Kendali Prediktif dan Adaptif, Universitas Indonesia

Np = p.Np;
b  = [0, -p.delta_T / p.E_nom];   % B = [b_grid, b_batt] = [0, -ΔT/E_nom]

Phi_x = ones(Np, 1);

Gamma_x = zeros(Np, 2 * Np);
for i = 1:Np
    for j = 1:i
        % Block (i,j) = B  (since A=1, A^(i-j) = 1)
        Gamma_x(i, 2*(j-1)+1 : 2*j) = b;
    end
end

Psi_x = zeros(Np, 3 * Np);         % no direct state effect in v1
end
