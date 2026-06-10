function [Phi_x, Gamma_x, Psi_x] = build_prediction_matrices(Np, dT, Enom)
%BUILD_PREDICTION_MATRICES Matriks prediksi bertumpuk untuk MPC microgrid.
%
% Tujuan:
%   Membangun matriks untuk prediksi keadaan bertumpuk:
%       X = Phi_x * x(k) + Gamma_x * U + Psi_x * D
%   dengan U = [ugrid(0); ubatt(0); ...; ugrid(Np-1); ubatt(Np-1)]  (2Np x 1)
%         D = [Ppv(0); Pload(0); c(0); ...; Ppv(Np-1); Pload(Np-1); c(Np-1)] (3Np x 1)
%
% Masukan:
%   Np   - horizon prediksi [langkah]
%   dT   - waktu sampling [jam]
%   Enom - energi nominal BESS [kWh]
%
% Keluaran:
%   Phi_x  - matriks Np x 1
%   Gamma_x - matriks Np x (2*Np) blok-Toeplitz segitiga-bawah
%   Psi_x  - matriks Np x (3*Np) (nol di v1)

Phi_x = ones(Np, 1);

%% Bangun Gamma_x: Np x 2Np
% Dengan A=1, prediksinya:
%   x(k+i) = x(k) + sum_{j=0}^{i-1} B*u(k+j)
% dengan B = [0, -dT/Enom]
% Gamma_x adalah matriks blok segitiga-bawah di mana setiap
% blok tak-nol (j<=i) = B = [0, -dT/Enom].
b = [0, -dT / Enom];
Gamma_x = zeros(Np, 2 * Np);

for row = 1:Np
    for block = 1:row
        col_start = (block - 1) * 2 + 1;
        Gamma_x(row, col_start:col_start+1) = b;
    end
end

%% Psi_x: gangguan tidak mempengaruhi SoC di v1
Psi_x = zeros(Np, 3 * Np);

end
