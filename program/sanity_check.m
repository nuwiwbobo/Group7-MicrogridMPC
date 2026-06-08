% sanity_check.m
% Quick end-to-end test of the Microgrid EMPC setup.
%
% Purpose:
%   Loads parameters, builds forecast and QP for x(0)=0.5, solves with
%   quadprog, and validates the result.

clear; clc;
fprintf('=== Microgrid EMPC Sanity Check ===\n');

params = parameters();
Np = params.Np;
[Phi_x, Gamma_x, ~] = build_prediction_matrices(Np, params.dT, params.Enom);
D_forecast = generate_forecasts(params);

x0 = params.x0;
D0 = D_forecast(1:Np, :);

[H, f] = build_qp(x0, D0, params, Phi_x, Gamma_x);
[A_eq, b_eq, A_ineq, b_ineq, lb, ub] = ...
    build_constraints(Np, params, x0, D0, Phi_x, Gamma_x);

options = optimoptions('quadprog', 'Display', 'off');
U_star = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, lb, ub, [], options);

ugrid0 = U_star(1);
ubatt0 = U_star(2);
J_opt = 0.5 * U_star' * H * U_star + f' * U_star;

fprintf('Optimal u*(0): ugrid = %.2f kW, ubatt = %.2f kW\n', ugrid0, ubatt0);
fprintf('Optimal cost J* = %.4f\n', J_opt);

if min(eig(H)) > 0
    fprintf('H is positive definite: YES\n');
else
    fprintf('H is positive definite: NO  (min eig = %e)\n', min(eig(H)));
end

% Check constraint satisfaction
eq_err = norm(A_eq * U_star - b_eq, inf);
ineq_viol = max(A_ineq * U_star - b_ineq);
if eq_err < 1e-8 && ineq_viol < 1e-8
    fprintf('All constraints satisfied: YES\n');
else
    fprintf('All constraints satisfied: NO  (eq_err=%.2e, ineq_viol=%.2e)\n', ...
        eq_err, ineq_viol);
end

if all(isfinite(U_star)) && all(isreal(U_star))
    fprintf('Sanity check PASSED.\n');
else
    fprintf('Sanity check FAILED: u_star has non-finite or complex values.\n');
end
