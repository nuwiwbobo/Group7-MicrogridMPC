function D_forecast = generate_forecasts(params)
%GENERATE_FORECASTS Generates 24-hour forecasts from real datasets.
%
% Data sources:
%   PV: PVGIS-ERA5 annual average hourly G(i) on 15-deg tilt, Depok
%       (-6.36, 106.83), 2020.  Ppv = G(i)/1000 * Ppv_installed * eta.
%   Load: IEEE RTS-GMLC day-ahead forecast, Region 1, July 15 (summer peak).
%         Normalised to Pload_peak for microgrid scale.
%   Price: Ontario Energy Board ULO TOU tariff, Nov 2025 (10x spread).
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

%% 3. Electricity price — Ontario Energy Board ULO TOU, Nov 2025
% Source: oeb.ca/newsroom/2025 (Ontario Energy Board, regulated tariff)
% Ultra-Low Overnight rate plan — chosen for large price spread (10x)
%   that creates strong battery arbitrage incentive in MPC results.
%   Weekday schedule (hours 0-23):
%     23-06: Ultra-Low Overnight  3.9 ¢/kWh
%     07-15: Mid-Peak            15.7 ¢/kWh
%     16-20: On-Peak             39.1 ¢/kWh
%     21-22: Mid-Peak            15.7 ¢/kWh
% Convert to $/kWh: ¢ → $ divide by 100.
c = zeros(24, 1);
for k = 1:24
    hour = k - 1;  % 0-indexed
    if hour >= 23 || hour <= 6
        c(k) = 0.039;   % ultra-low overnight
    elseif hour >= 7 && hour <= 15
        c(k) = 0.157;   % mid-peak
    elseif hour >= 16 && hour <= 20
        c(k) = 0.391;   % on-peak
    else  % hours 21-22
        c(k) = 0.157;   % mid-peak
    end
end

D_forecast = [Ppv, Pload, c];

end
