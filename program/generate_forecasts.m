function D_forecast = generate_forecasts(params)
%GENERATE_FORECASTS Membangkitkan prakiraan 24 jam dari dataset nyata.
%
% Sumber data:
%   PV: PVGIS-ERA5 rata-rata tahunan G(i) per jam, kemiringan 15°, Depok
%       (-6.36, 106.83), 2020.  Ppv = G(i)/1000 * Ppv_terpasang * eta.
%   Beban: IEEE RTS-GMLC prakiraan sehari-sebelumnya, Region 1, 15 Juli.
%          Dinormalisasi ke Pload_puncak untuk skala microgrid.
%   Harga: Ontario Energy Board ULO TOU, Nov 2025 (selisih 10x).
%
% Lihat juga: parameters

Ppv_installed = params.Ppv_installed;
Pload_peak    = params.Pload_peak;

%% 1. Daya PV dari PVGIS rata-rata tahunan G(i) kemiringan 15° menghadap utara
% G(i) per jam [W/m²], UTC → WIB = UTC+7, jadi jam 0 = 07:00 lokal
% Efisiensi sistem ~85% (inverter + kabel + suhu + kotoran)
eta_system = 0.85;
ghi_avg = [ ...
    217.0; 394.2; 540.3; 625.8; 641.6; 612.7; ...  % jam 0-5
    518.8; 397.6; 260.8; 131.8;  29.1;   0.0; ...  % jam 6-11
      0.0;   0.0;   0.0;   0.0;   0.0;   0.0; ...  % jam 12-17
      0.0;   0.0;   0.0;   0.0;   0.6;  64.2];     % jam 18-23
Ppv = ghi_avg / 1000 * Ppv_installed * eta_system;

%% 2. Profil beban dari IEEE RTS-GMLC (Region 1, 15 Juli, dinormalisasi)
% Dinormalisasi ke pu dari puncak (2652.9 MW). Lalu diskalakan ke Pload_puncak.
rts_pu = [ ...
    0.582; 0.551; 0.537; 0.538; 0.547; 0.584; ...  % jam 0-5
    0.626; 0.695; 0.741; 0.796; 0.843; 0.890; ...  % jam 6-11
    0.934; 0.969; 0.989; 1.000; 0.988; 0.958; ...  % jam 12-17
    0.918; 0.890; 0.846; 0.771; 0.704; 0.651];     % jam 18-23
Pload = rts_pu * Pload_peak;

%% 3. Harga listrik — Ontario Energy Board ULO TOU, Nov 2025
% Sumber: oeb.ca/newsroom/2025 (Ontario Energy Board, tarif diatur)
% Rencana Ultra-Low Overnight — dipilih karena selisih harga besar (10x)
%   yang menciptakan insentif arbitrase baterai kuat di hasil MPC.
%   Jadwal hari kerja (jam 0-23):
%     23-06: Ultra-Low Overnight  3.9 ¢/kWh
%     07-15: Mid-Peak            15.7 ¢/kWh
%     16-20: On-Peak             39.1 ¢/kWh
%     21-22: Mid-Peak            15.7 ¢/kWh
% Konversi ke $/kWh: ¢ → $ bagi 100.
c = zeros(24, 1);
for k = 1:24
    hour = k - 1;  % indeks 0
    if hour >= 23 || hour <= 6
        c(k) = 0.039;   % ultra-low semalam
    elseif hour >= 7 && hour <= 15
        c(k) = 0.157;   % menengah
    elseif hour >= 16 && hour <= 20
        c(k) = 0.391;   % puncak
    else  % jam 21-22
        c(k) = 0.157;   % menengah
    end
end

D_forecast = [Ppv, Pload, c];

end
