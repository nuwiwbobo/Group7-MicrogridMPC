function params = parameters()
%PARAMETERS Mengembalikan semua parameter sistem untuk proyek EMPC Microgrid.
%
% Tujuan:
%   Definisi parameter terpusat untuk microgrid terhubung-jaringan dengan PV,
%   BESS, beban, dan impor jaringan.
%
% Keluaran:
%   params - struct berisi semua parameter numerik
%
% Satuan & konvensi tanda:
%   ubatt > 0 -> discharging (SoC turun)
%   ugrid > 0 -> impor dari jaringan
%   Ppv >= 0, Pload >= 0

%% Parameter BESS
params.Enom      = 5.0;    % Kapasitas energi nominal BESS  [kWh]
params.Pbatt_max = 2.5;    % Daya max BESS (charge/discharge) [kW]
params.SoCmin    = 0.2;    % State of Charge minimum [-]
params.SoCmax    = 0.8;    % State of Charge maksimum [-]
params.x0        = 0.5;    % State of Charge awal [-]
params.xtarget   = 0.5;    % Target State of Charge terminal [-]

%% Parameter Jaringan
params.Pgrid_max = 10.0;   % Daya max impor jaringan [kW]

%% Parameter PV & Beban
params.Ppv_installed = 3.0; % Kapasitas terpasang PV [kWp]
params.Pload_peak    = 4.0; % Daya beban puncak [kW]

%% Parameter MPC
params.dT  = 1.0;   % Waktu sampling [jam]
params.Np  = 24;    % Horizon prediksi [langkah]
params.lambda  = 1.0;   % Bobot biaya terminal [-]
params.epsilon = 1e-3;  % Bobot regularisasi Tikhonov [-]

end
