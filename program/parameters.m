function p = parameters()
%PARAMETERS  Microgrid plant-model parameters for EMPC.
%   P = PARAMETERS() returns a struct with all physical and tuning
%   parameters.  Call once at the top of any script that needs them.
%
%   Example:
%       p = parameters();
%       fprintf('Battery capacity = %.2f kWh\n', p.E_nom);
%
% References
%   Cortes-Aguirre et al. 2024 (arXiv 2412.10851)
%   Vasilj et al. 2019 (IEEE TSG 10(2):1992--2001)
%
% Group 7 - Microgrid Economic MPC
% Sistem Kendali Prediktif dan Adaptif, Universitas Indonesia

% ----- Physical parameters -----
p.E_nom         = 5.0;      % BESS nominal energy           [kWh]
p.P_batt_max    = 2.5;      % BESS max (dis)charge power    [kW]
p.P_grid_max    = 10.0;     % grid import limit             [kW]
p.P_pv_peak     = 3.0;      % PV installed capacity         [kWp]
p.P_load_peak   = 4.0;      % peak load estimate            [kW]

% ----- Battery limits -----
p.SoC_min       = 0.2;      % lower bound                   [-]
p.SoC_max       = 0.8;      % upper bound                   [-]
p.x0            = 0.5;      % initial state-of-charge       [-]

% ----- Time -----
p.delta_T       = 1.0;      % sampling interval             [h]

% ----- MPC tuning -----
p.Np            = 24;       % prediction horizon            [steps]
p.x_target      = 0.5;      % desired SoC at horizon end    [-]
p.lambda        = 1.0;      % terminal cost weight          [$/(kWh)^2]
p.epsilon       = 1e-3;     % Tikhonov regularizer          [-]
end
