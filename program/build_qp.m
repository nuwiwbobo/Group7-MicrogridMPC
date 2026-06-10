function [H, f] = build_qp(x_k, D_horizon, params, Phi_x, Gamma_x)
%BUILD_QP Membangun matriks biaya QP (H dan f) untuk Economic MPC.
%
% Tujuan:
%   Membangun biaya kuadratik:
%       J = sum_{k=0}^{Np-1} [ c(k)*ugrid(k)*dT + epsilon*||u(k)||^2 ]
%           + lambda*(x(Np) - xtarget)^2
%
%   Setelah substitusi X = Phi_x*x0 + Gamma_x*U + Psi_x*D:
%       H = Hq + Hr  (2Np x 2Np)
%       f = fq + fl  (2Np x 1)
%
% Masukan:
%   x_k       - keadaan saat ini (SoC) [-]
%   D_horizon - matriks Np x 3 [Ppv, Pload, c]
%   params    - struct dengan field: dT, lambda, epsilon, xtarget
%   Phi_x     - matriks prediksi Np x 1
%   Gamma_x   - matriks prediksi Np x (2*Np)
%
% Keluaran:
%   H - matriks Hessian 2Np x 2Np (harus definit positif)
%   f - vektor gradien 2Np x 1

Np      = size(D_horizon, 1);
dT      = params.dT;
lambda  = params.lambda;
epsilon = params.epsilon;
xtarget = params.xtarget;

%% Biaya terminal (kuadratik)
% Hq = 2 * lambda * Gamma_x' * Gamma_x
Hq = 2 * lambda * (Gamma_x' * Gamma_x);

%% Regularisasi Tikhonov
% Hr = 2 * epsilon * eye(2*Np)
Hr = 2 * epsilon * eye(2 * Np);

H = Hq + Hr;

%% Suku linear dari biaya terminal
% fq' = 2 * lambda * (Phi_x * x_k - xtarget)' * Gamma_x
% fq  = fq'  (vektor kolom)
fq = (2 * lambda * (Phi_x * x_k - xtarget)' * Gamma_x)';

%% Suku linear dari biaya ekonomi
% fl = cfull * dT dengan cfull = [c(0); 0; c(1); 0; ...; c(Np-1); 0]  (2Np x 1)
cfull = zeros(2 * Np, 1);
for k = 1:Np
    cfull((k - 1) * 2 + 1) = D_horizon(k, 3);  % c(k) di posisi ugrid
end
fl = cfull * dT;

f = fq + fl;

%% Verifikasi H definit positif
eigH = eig(H);
if min(eigH) > 0
    fprintf('H definit positif (min eig = %e)\n', min(eigH));
else
    warning('H TIDAK definit positif! Nilai eigen min = %e', min(eigH));
end

end
