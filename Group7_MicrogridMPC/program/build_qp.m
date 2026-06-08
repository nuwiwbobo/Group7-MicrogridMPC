function [H, f] = build_qp(x_k, D_horizon, params, Phi_x, Gamma_x)
%BUILD_QP Builds QP cost matrices H and f for Economic MPC.
%
% Purpose:
%   Constructs the quadratic cost:
%       J = sum_{k=0}^{Np-1} [ c(k)*ugrid(k)*dT + epsilon*||u(k)||^2 ]
%           + lambda*(x(Np) - xtarget)^2
%
%   After substituting X = Phi_x*x0 + Gamma_x*U + Psi_x*D:
%       H = Hq + Hr  (both 2Np x 2Np)
%       f = fq + fl  (2Np x 1)
%
% Inputs:
%   x_k       - current state (SoC) [-]
%   D_horizon - Np x 3 matrix [Ppv, Pload, c]
%   params    - struct with fields: dT, lambda, epsilon, xtarget
%   Phi_x     - Np x 1 prediction matrix
%   Gamma_x   - Np x (2*Np) prediction matrix
%
% Outputs:
%   H - 2Np x 2Np Hessian matrix (must be positive definite)
%   f - 2Np x 1 gradient vector

Np      = size(D_horizon, 1);
dT      = params.dT;
lambda  = params.lambda;
epsilon = params.epsilon;
xtarget = params.xtarget;

%% Terminal cost (quadratic)
% Hq = 2 * lambda * Gamma_x' * Gamma_x
Hq = 2 * lambda * (Gamma_x' * Gamma_x);

%% Tikhonov regularisation
% Hr = 2 * epsilon * eye(2*Np)
Hr = 2 * epsilon * eye(2 * Np);

H = Hq + Hr;

%% Linear term from terminal cost
% fq' = 2 * lambda * (Phi_x * x_k - xtarget)' * Gamma_x
% fq  = fq'  (i.e., column vector)
fq = (2 * lambda * (Phi_x * x_k - xtarget)' * Gamma_x)';

%% Linear term from economic cost
% fl = cfull * dT where cfull = [c(0); 0; c(1); 0; ...; c(Np-1); 0]  (2Np x 1)
cfull = zeros(2 * Np, 1);
for k = 1:Np
    cfull((k - 1) * 2 + 1) = D_horizon(k, 3);  % c(k) at ugrid positions
end
fl = cfull * dT;

f = fq + fl;

%% Verify H is positive definite
eigH = eig(H);
if min(eigH) > 0
    fprintf('H is positive definite (min eig = %e)\n', min(eigH));
else
    warning('H is NOT positive definite! Min eigenvalue = %e', min(eigH));
end

end
