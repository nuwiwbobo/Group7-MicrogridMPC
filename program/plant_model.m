function [A, B, C, D_ss, E, Fe] = plant_model(params)
%PLANT_MODEL Mengembalikan matriks state-space LTI untuk plant microgrid.
%
% Tujuan:
%   Mendefinisikan model state-space waktu-diskrit:
%       x(k+1) = A*x(k) + B*u(k) + E*d(k)
%       y(k)   = C*x(k) + D_ss*u(k) + Fe*d(k)
%
% Masukan:
%   params - struct dengan field: Enom, dT
%
% Keluaran:
%   A, B, C, D_ss, E, Fe - matriks state-space
%
% Satuan & konvensi tanda:
%   x  = SoC  [-]  (tak-berdimensi, 0-1)
%   u  = [ugrid; ubatt]  [kW]
%   d  = [Ppv; Pload; c]  [kW, kW, $/kWh]
%   y  = [ugrid; x]       [kW, -]

dT   = params.dT;
Enom = params.Enom;

A = 1;
B = [0, -dT / Enom];
E = [0, 0, 0];

C = [0; 1];
D_ss = [1, -1; 0, 0];
Fe = [-1, 1, 0; 0, 0, 0];

%% Cek dimensi matriks
fprintf('Dimensi model plant:\n');
fprintf('  A:  %d x %d\n', size(A,1), size(A,2));
fprintf('  B:  %d x %d\n', size(B,1), size(B,2));
fprintf('  C:  %d x %d\n', size(C,1), size(C,2));
fprintf('  D:  %d x %d\n', size(D_ss,1), size(D_ss,2));
fprintf('  E:  %d x %d\n', size(E,1), size(E,2));
fprintf('  Fe: %d x %d\n', size(Fe,1), size(Fe,2));

end
