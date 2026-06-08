function [Phi_x, Gamma_x, Psi_x] = build_prediction_matrices(Np, dT, Enom)
%BUILD_PREDICTION_MATRICES Stacked prediction matrices for the microgrid MPC.
%
% Purpose:
%   Constructs the matrices for the stacked state prediction:
%       X = Phi_x * x(k) + Gamma_x * U + Psi_x * D
%   where U = [ugrid(0); ubatt(0); ...; ugrid(Np-1); ubatt(Np-1)]  (2Np x 1)
%         D = [Ppv(0); Pload(0); c(0); ...; Ppv(Np-1); Pload(Np-1); c(Np-1)] (3Np x 1)
%
% Inputs:
%   Np   - prediction horizon [steps]
%   dT   - sampling time [hour]
%   Enom - BESS nominal energy [kWh]
%
% Outputs:
%   Phi_x  - Np x 1 matrix
%   Gamma_x - Np x (2*Np) lower-triangular block-Toeplitz matrix
%   Psi_x  - Np x (3*Np) matrix (zeros in v1)

Phi_x = ones(Np, 1);

%% Build Gamma_x: Np x 2Np
% With A=1, the prediction is:
%   x(k+i) = x(k) + sum_{j=0}^{i-1} B*u(k+j)
% where B = [0, -dT/Enom]
% Gamma_x is a lower-triangular block matrix where each
% non-zero block (j<=i) equals B = [0, -dT/Enom].
b = [0, -dT / Enom];
Gamma_x = zeros(Np, 2 * Np);

for row = 1:Np
    for block = 1:row
        col_start = (block - 1) * 2 + 1;
        Gamma_x(row, col_start:col_start+1) = b;
    end
end

%% Psi_x: disturbances do not affect SoC in v1
Psi_x = zeros(Np, 3 * Np);

end
