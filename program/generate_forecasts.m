function D_forecast = generate_forecasts(params)
%GENERATE_FORECASTS Generates 24-hour forecasts from real datasets.
%
% Data sources:
%   PV: PVGIS-ERA5 annual average hourly G(i) on 15-deg tilt, Depok
%       (-6.36, 106.83), 2020.  Ppv = G(i)/1000 * Ppv_installed * eta.
%   Load: IEEE RTS-GMLC day-ahead forecast, Region 1, July 15 (summer peak).
%         Normalised to Pload_peak for microgrid scale.
%   Price: PLN flat residential tariff R-1 (1300-2200 VA), June 2025.
%
% See also: parameters

Ppv_installed = params.Ppv_installed;
Pload_peak    = params.Pload_peak;

%% 1. PV power from PVGIS annual average G(i) on 15-deg north-facing tilt
% Hourly G(i) [W/m²], UTC → WIB = UTC+7, so hour 0 = 07:00 local
% System efficiency ~85% (inverter + wiring + temperature + soiling)
eta_system = 0.85;
ghi_avg = [ ...
    217.0; 394.2; 540.3; 625.8; 641.6; 612.7; ...  % hours 0-5
    518.8; 397.6; 260.8; 131.8;  29.1;   0.0; ...  % hours 6-11
      0.0;   0.0;   0.0;   0.0;   0.0;   0.0; ...  % hours 12-17
      0.0;   0.0;   0.0;   0.0;   0.6;  64.2];     % hours 18-23
Ppv = ghi_avg / 1000 * Ppv_installed * eta_system;

%% 2. Load profile from IEEE RTS-GMLC (Region 1, July 15, normalised)
% Normalised to pu of peak (2652.9 MW).  Then scaled to Pload_peak.
rts_pu = [ ...
    0.582; 0.551; 0.537; 0.538; 0.547; 0.584; ...  % hours 0-5
    0.626; 0.695; 0.741; 0.796; 0.843; 0.890; ...  % hours 6-11
    0.934; 0.969; 0.989; 1.000; 0.988; 0.958; ...  % hours 12-17
    0.918; 0.890; 0.846; 0.771; 0.704; 0.651];     % hours 18-23
Pload = rts_pu * Pload_peak;

%% 3. Electricity price — PLN flat tariff R-1 (1300-2200 VA), June 2025
% Rp 1,444.70/kWh ≈ USD 0.09/kWh at 16,000 IDR/USD
% No TOU for residential in Indonesia; set TOU flag to false in params
%   if you want TOU for business (B-3), see comments below.
c = 0.09 * ones(24, 1);  % flat, all hours

% Uncomment for PLN B-3 business TOU (peak 17:00-22:00):
% c(:) = 0.09;
% for k = 1:24
%     hour = k - 1;  % 0-indexed
%     if hour >= 17 && hour <= 22
%         c(k) = 0.15;  % peak ~1.67x
%     end
% end

D_forecast = [Ppv, Pload, c];

end
