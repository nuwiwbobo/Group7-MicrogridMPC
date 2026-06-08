function D_forecast = generate_forecasts(params)
%GENERATE_FORECASTS Generates 24-hour synthetic forecasts.
%
% Purpose:
%   Creates synthetic 24-hour profiles for PV generation, load demand,
%   and electricity price (TOU tariff).
%
% Inputs:
%   params - struct with fields: Ppv_installed, Pload_peak
%
% Outputs:
%   D_forecast - 24 x 3 matrix = [Ppv, Pload, c]
%                Columns: PV power [kW], Load power [kW], Price [$/kWh]
%
% Sign convention: Ppv >= 0, Pload >= 0

Ppv_installed = params.Ppv_installed;
Pload_peak    = params.Pload_peak;

Ppv = zeros(24, 1);
for k = 1:24
    hour = k - 1;
    if hour >= 6 && hour <= 18
        Ppv(k) = Ppv_installed * max(0, sin(pi * (hour - 6) / 12));
    end
end

Pload = zeros(24, 1);
for k = 1:24
    hour = k - 1;
    if hour >= 0 && hour <= 5 || hour >= 22 && hour <= 23
        Pload(k) = 0.5 * Pload_peak;
    elseif hour >= 6 && hour <= 9
        Pload(k) = 0.8 * Pload_peak + 0.2 * Pload_peak * sin(pi * (hour - 6) / 4);
    elseif hour >= 10 && hour <= 16
        Pload(k) = 0.4 * Pload_peak;
    elseif hour >= 17 && hour <= 21
        Pload(k) = 0.9 * Pload_peak + 0.1 * Pload_peak * sin(pi * (hour - 17) / 5);
    end
end

%% TOU electricity price
c_base = 0.10; % $/kWh
c = zeros(24, 1);
for k = 1:24
    hour = k - 1;
    if hour >= 0 && hour <= 17 || hour >= 22 && hour <= 23
        c(k) = c_base;
    elseif hour >= 18 && hour <= 21
        c(k) = 2.5 * c_base;
    end
end

D_forecast = [Ppv, Pload, c];

end
