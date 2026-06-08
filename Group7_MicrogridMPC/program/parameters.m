function params = parameters()
%PARAMETERS Returns all system parameters for the Microgrid EMPC project.
%
% Purpose:
%   Centralized parameter definition for the grid-tied microgrid with PV,
%   BESS, load, and grid import (v1: no export).
%
% Returns:
%   params - struct containing all numeric parameters
%
% Units & sign convention:
%   ubatt > 0 -> discharging (SoC decreases)
%   ugrid > 0 -> importing from grid
%   Ppv >= 0, Pload >= 0

%% BESS parameters
params.Enom      = 5.0;    % BESS nominal energy capacity  [kWh]
params.Pbatt_max = 2.5;    % BESS max power (charge/discharge) [kW]
params.SoCmin    = 0.2;    % Minimum State of Charge [-]
params.SoCmax    = 0.8;    % Maximum State of Charge [-]
params.x0        = 0.5;    % Initial State of Charge [-]
params.xtarget   = 0.5;    % Target terminal State of Charge [-]

%% Grid parameters
params.Pgrid_max = 10.0;   % Max grid import power [kW]

%% PV & Load parameters
params.Ppv_installed = 3.0; % PV installed capacity [kWp]
params.Pload_peak    = 4.0; % Peak load power [kW]

%% MPC parameters
params.dT  = 1.0;   % Sampling time [hour]
params.Np  = 24;    % Prediction horizon [steps]
params.lambda  = 1.0;   % Terminal cost weight [-]
params.epsilon = 1e-3;  % Tikhonov regularisation weight [-]

end
