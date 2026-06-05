# Microgrid Economic MPC — Plant + System Modeling Design

**Course:** Sistem Kendali Prediktif dan Adaptif (Predictive & Adaptive Control Systems)
**University:** Universitas Indonesia
**Project theme:** Project 3 — Microgrid Economic MPC
**Group:** 7 (Danish Rafi Akhmadia, Ahmad Farrij Dzulfikkar, Kholifah Zein Nuril Musthafa, Nawaf Dhia Alwafa Dipatama)
**Owner of this piece:** Kholifah Zein Nuril Musthafa (2306263664)
**Scope:** Plant + system modeling (Section 1.2 items 1, 2, 3 of the assignment; Sections 1, 2, 3 of Section 2 deliverable structure)
**Status:** Design — awaiting approval
**Date:** 2026-06-05
**Deadline:** 2026-06-10 23:59 WIB (5 days from now)
**Spec author:** GSD brainstorming flow (Kholifah + research synthesis)

---

## 0. TL;DR

A **discrete-time linear time-invariant (LTI) state-space plant model** for a grid-tied microgrid with PV, BESS, and load, designed to be the input to a downstream Economic MPC (EMPC) formulation. The state is the battery state-of-charge (SoC, scalar). Controls are grid import and battery power. Disturbances are PV power, load demand, and electricity price (perfect forecasts, v1). Time scale is 1 hour, prediction horizon 24 hours. The objective function (formulated by the MPC-formulation teammate) is a linear-in-control stage cost `c(k)·u_grid(k)·ΔT` with a quadratic terminal cost on SoC deviation and a tiny Tikhonov regularizer — this makes the downstream optimization a convex QP solvable with MATLAB `quadprog` in milliseconds. All hand-off code is documented in a README contract so the MPC-formulation, simulation, and report teammates can integrate without further negotiation.

---

## 1. System Architecture

The modeled microgrid is a 4-component system on a common AC bus, grid-tied:

```
                   ┌──────────────┐
   P_pv(k) ──────▶│              │
                   │   AC Bus    │◀──── P_load(k)
   u_batt(k) ────▶│  (Modeled)  │      (forecast
   (charge/disch) │              │       disturbance)
                   └──────┬───────┘
                          │
                          ▼
                       u_grid(k)
                    (import from
                       utility)
```

- **PV array:** forecasted, curtailable (v1: no explicit curtailment; see §10 risk).
- **BESS (battery):** 1 dynamic state (SoC). 1 control input (charge/discharge power).
- **Load:** forecasted, exogenous.
- **Grid:** controlled import. Export is not allowed in v1 (flagged as v2).

**Sign convention** (matches Cortes-Aguirre et al. 2024, arXiv 2412.10851):
- `u_batt > 0` = **discharging** (battery delivers to AC bus; SoC decreases).
- `u_batt < 0` = **charging** (battery absorbs from AC bus; SoC increases).
- `u_grid > 0` = **importing** from grid.
- `u_grid < 0` = **exporting** to grid (NOT allowed in v1; lower bound = 0).
- `P_pv ≥ 0`, `P_load ≥ 0`.

---

## 2. State-Space Plant Model

This is the central deliverable. The matrices `(A, B, C, D, E, F_e)` are passed to the MPC-formulation teammate.

### 2.1 Variables and dimensions

| Symbol | Meaning | Dimension | Unit |
|---|---|---|---|
| `x(k)` | Battery state-of-charge (SoC), normalized | ℝ¹ | dimensionless [0, 1] |
| `u(k) = [u_grid(k), u_batt(k)]ᵀ` | Control inputs | ℝ² | kW |
| `d(k) = [P_pv(k), P_load(k), c(k)]ᵀ` | Exogenous disturbances (forecast, perfect in v1) | ℝ³ | kW, kW, $/kWh |
| `y(k) = [u_grid(k), x(k)]ᵀ` | Measured/controlled outputs | ℝ² | kW, dimensionless |
| `ΔT` | Sampling time | scalar | 1 h |

### 2.2 State equation

$$x(k+1) = A \cdot x(k) + B \cdot u(k) + E \cdot d(k)$$

Matrices (scalar `A`, row `B`, row `E`):

$$A = \begin{bmatrix} 1 \end{bmatrix}, \qquad B = \begin{bmatrix} 0 & -\dfrac{\Delta T}{E_{\text{nom}}} \end{bmatrix}, \qquad E = \begin{bmatrix} 0 & 0 & 0 \end{bmatrix}$$

**Derivation of B:** In one hour, discharging at `u_batt > 0` removes energy `u_batt · ΔT` from the battery. Normalized: `Δx = −(u_batt · ΔT) / E_nom`. SoC is `x(k+1) = x(k) + Δx = x(k) − (u_batt · ΔT) / E_nom`. This gives `B = [0, −ΔT/E_nom]`. Grid import `u_grid` does not directly affect SoC. Disturbances do not directly affect SoC in v1.

**Efficiency simplification (v1):** Round-trip efficiency is 100% in the dynamics. The penalty for losses appears only in the cost (see §4) to keep dynamics linear and the QP convex. (This is Cortes-Aguirre 2024's deliberate choice, §II-D, justified as "preserves convexity.") Asymmetric `η_c ≠ η_d` is flagged for v2.

### 2.3 Output equation

$$y(k) = C \cdot x(k) + D \cdot u(k) + F_e \cdot d(k)$$

$$C = \begin{bmatrix} 0 \\ 1 \end{bmatrix}, \qquad D = \begin{bmatrix} 1 & -1 \\ 0 & 0 \end{bmatrix}, \qquad F_e = \begin{bmatrix} -1 & 1 & 0 \\ 0 & 0 & 0 \end{bmatrix}$$

**Derivation of D and F_e (output 1 = grid import):** From power balance, `P_load = P_pv + u_batt + u_grid`, so `u_grid = P_load − P_pv − u_batt`. Output 1 is `y_1 = u_grid = −u_batt + P_load − P_pv`, giving `D` row 1 = `[1, −1]` (controls) and `F_e` row 1 = `[−1, 1, 0]` (disturbances: `−P_pv` for column 1, `+P_load` for column 2, `0` for price column 3). Output 2 is `y_2 = x`, the SoC.

### 2.4 Power-balance equality constraint

Embedded in the output equation but also listed as a QP equality constraint for clarity:

$$P_{\text{load}}(k) - P_{\text{pv}}(k) - u_{\text{batt}}(k) - u_{\text{grid}}(k) = 0 \quad \forall k \in \{0, \dots, N_p - 1\}$$

This eliminates `u_grid` from the free decision variables if the MPC-formulation teammate prefers; the choice is theirs. Recommended: keep both `u_grid` and `u_batt` as free controls and add this as an equality, so the cost `c(k)·u_grid(k)·ΔT` directly captures the electricity bill.

---

## 3. Prediction Matrices F and Φ (assignment Section 1.2 item 3)

The standard stacked MPC prediction form (Rawlings, Mayne, Diehl 2017, Chapter 2). **This is the form the assignment explicitly asks for** ("membentuk matriks prediksi F dan Φ"). None of the 4 papers we surveyed use this form (Cortes-Aguirre uses CVX directly, Polanco Vásquez uses algebraic constraints, Vasilj uses MILP, Jia uses ADMM iterations) — but the form is equivalent, and the assignment requires it.

### 3.1 Stacked vectors

| Vector | Definition | Dimension |
|---|---|---|
| `X` | `[x(k+1\|k), x(k+2\|k), …, x(k+N_p\|k)]ᵀ` | ℝ^{N_p} |
| `U` | `[u(k\|k), u(k+1\|k), …, u(k+N_p-1\|k)]ᵀ` (interleaved grid/batt) | ℝ^{2 N_p} |
| `D` | `[d(k\|k), d(k+1\|k), …, d(k+N_p-1\|k)]ᵀ` (interleaved pv/load/price) | ℝ^{3 N_p} |

### 3.2 Stacked prediction equation

$$X = \Phi_x \cdot x(k) + \Gamma_x \cdot U + \Psi_x \cdot D$$

For our model (A = 1, B = [0, −ΔT/E_nom], E = 0):

- **Φ_x = [1, 1, …, 1]ᵀ ∈ ℝ^{N_p}** (column of N_p ones — every future state starts from the current one with gain 1).
- **Γ_x** is lower-triangular block-Toeplitz. Row i (i = 1, …, N_p) is:
  $$\begin{bmatrix} 0 & -\dfrac{i \cdot \Delta T}{E_{\text{nom}}} \end{bmatrix}$$
  Stacked for all 2N_p columns over all N_p rows, with the (i, j) entry of the sub-block being `(i − j + 1) · b` for `j ≤ i`, else 0, where `b = [0, −ΔT/E_nom]`.
- **Ψ_x = 0** (disturbances do not affect state in v1).

### 3.3 Code to construct Γ_x

Provided to the MPC-formulation teammate as `build_prediction_matrices.m`:

```matlab
function [Phi_x, Gamma_x, Psi_x] = build_prediction_matrices(Np, dT, E_nom)
    % Construct the stacked prediction matrices for the microgrid plant.
    %
    % Inputs:
    %   Np    - prediction horizon (e.g. 24)
    %   dT    - sampling time in hours (e.g. 1.0)
    %   E_nom - BESS nominal energy in kWh (e.g. 5.0)
    %
    % Outputs:
    %   Phi_x  - Np x 1   (state contribution)
    %   Gamma_x - Np x 2Np (control contribution)
    %   Psi_x  - Np x 3Np (disturbance contribution, zero in v1)

    Phi_x = ones(Np, 1);
    b = [0, -dT / E_nom];          % 1x2 row of B
    Gamma_x = zeros(Np, 2*Np);
    for i = 1:Np
        for j = 1:i
            Gamma_x(i, 2*(j-1)+1 : 2*j) = (i - j + 1) * b;
        end
    end
    Psi_x = zeros(Np, 3*Np);
end
```

---

## 4. Economic Objective (the cost the QP minimizes)

The MPC-formulation teammate constructs the QP cost `J = ½ Uᵀ H U + fᵀ U` from these ingredients. Three terms:

### 4.1 Stage cost (linear in control — drives "economic" behavior)

$$L(k) = c(k) \cdot u_{\text{grid}}(k) \cdot \Delta T$$

Interpretation: at hour `k`, if you import `u_grid` kW and the price is `c(k)` $/kWh, the bill is `c(k) · u_grid(k) · 1 h`. Multiply by ΔT (in hours) to get kWh.

### 4.2 Terminal cost (quadratic — keeps the SoC reachable at horizon end)

$$V_f(x(N_p)) = \lambda \cdot (x(N_p) - x_{\text{target}})^2$$

`x_target = 0.5` (return battery to half charge by end of horizon). `λ > 0` is a tuning parameter (recommend `λ = 1.0` for v1; sweep ±10× in analysis).

**Origin:** This is the single-layer simplification of Vasilj et al. 2019's two-layer "cost-to-go `V_s`" and Cortes-Aguirre 2024's terminal cost `V_f` (their Eq. 12, more complex). Our simple quadratic keeps the QP well-posed.

### 4.3 Tikhonov regularization (quadratic — makes the QP well-conditioned)

$$R(u) = \varepsilon \cdot \|u(k)\|^2$$

`ε ≈ 1e-3` (very small). Physical meaning: penalize sudden swings in battery/grid power to keep the closed-loop trajectory smooth.

### 4.4 Total cost

$$J = \sum_{k=0}^{N_p - 1} \left[ c(k) \cdot u_{\text{grid}}(k) \cdot \Delta T + \varepsilon \cdot \|u(k)\|^2 \right] + \lambda \cdot (x(N_p) - x_{\text{target}})^2$$

### 4.5 QP matrices (H, f) the teammate builds

After substituting `X = Φ_x·x(k) + Γ_x·U`:

- From the terminal cost: `H_q = 2 λ Γ_xᵀ Γ_x`, `f_q = 2 λ (Φ_x·x(k) − x_target)ᵀ Γ_x`
- From regularization: `H_r = 2 ε I_{2N_p}`, `f_r = 0`
- From stage cost: define `c_full` as the 2N_p × 1 vector `[c(0), 0, c(1), 0, …, c(N_p−1), 0]ᵀ · ΔT`. Then `f_l = c_full`, `H_l = 0`.

**Total:** `H = H_q + H_r` (must be positive definite; `ε` ensures this), `f = f_q + f_l`.

This is a **convex QP** that MATLAB's `quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, lb, ub)` solves in milliseconds.

### 4.6 v2 extensions (out of v1 scope, mentioned for context)

- **Peak shaving:** add `α · max(0, u_grid(k) − P_peak_threshold)²` per hour. Needs slack or epigraph reformulation to keep QP.
- **Renewable utilization:** add `β · P_pv_curtailed(k)²` if v1 is extended to allow PV curtailment.
- **Battery degradation:** add `γ · |u_batt(k)|` (cycle-cost) or `γ · (u_batt(k))²` (loss proxy).
- **Multi-objective (Polanco Vásquez 2023):** add `δ · CO₂(P_grid, P_DG, ...)` — needs emission factors and possibly a generator state. Probably not in scope for v1.
- **Time-varying `λ`, `ε`:** could be made horizon-dependent (tighter `ε` at start, looser at end).

---

## 5. Constraints

All in QP form. The MPC-formulation teammate builds the matrices; we just specify them.

| # | Type | Constraint | Mathematical form | QP form |
|---|---|---|---|---|
| 1 | Equality | Power balance | `u_grid(k) − u_batt(k) = P_load(k) − P_pv(k)` | `A_eq · U = b_eq` |
| 2 | Inequality | SoC lower bound | `x(k) ≥ SoC_min` for `k = 1, …, N_p` | `A_ineq · U ≤ b_ineq` |
| 3 | Inequality | SoC upper bound | `x(k) ≤ SoC_max` for `k = 1, …, N_p` | `A_ineq · U ≤ b_ineq` |
| 4 | Box | Battery power | `−P_batt_max ≤ u_batt(k) ≤ P_batt_max` | `lb ≤ U ≤ ub` |
| 5 | Box | Grid import | `0 ≤ u_grid(k) ≤ P_grid_max` (no export in v1) | `lb ≤ U ≤ ub` |
| 6 | Inequality (optional) | Terminal SoC | `x(N_p) ≥ SoC_min_terminal` (e.g. 0.3) | `A_ineq · U ≤ b_ineq` |

The SoC constraints (#2, #3) are linear inequalities on `U` after substitution `X = Φ_x·x(k) + Γ_x·U`. Specifically:
- Lower: `Γ_x · U ≥ SoC_min · 1 − Φ_x · x(k)` → `−Γ_x · U ≤ Φ_x · x(k) − SoC_min · 1`
- Upper: `Γ_x · U ≤ SoC_max · 1 − Φ_x · x(k)` → `Γ_x · U ≤ SoC_max · 1 − Φ_x · x(k)`

**Code to build constraints** is provided as `build_constraints.m` (see §9).

---

## 6. Parameters (placeholder values; to be confirmed by team)

| Parameter | Symbol | Value | Unit | Notes |
|---|---|---|---|---|
| BESS nominal energy | `E_nom` | 5.0 | kWh | Lab-scale lithium. Replace with team's actual spec. |
| BESS max power | `P_batt_max` | 2.5 | kW | 0.5C rate |
| Grid max import | `P_grid_max` | 10.0 | kW | UI lab grid connection |
| PV installed | `P_pv_installed` | 3.0 | kWp | UI rooftop estimate |
| Load peak | `P_load_peak` | 4.0 | kW | UI building scale |
| SoC min | `SoC_min` | 0.2 | – | Lithium longevity |
| SoC max | `SoC_max` | 0.8 | – | Lithium longevity |
| Initial SoC | `x(0)` | 0.5 | – | Midpoint |
| Sampling time | `ΔT` | 1.0 | h | Matches assignment plot granularity |
| Prediction horizon | `N_p` | 24 | hours | Matches assignment plot length |
| Target SoC at horizon end | `x_target` | 0.5 | – | Midpoint |
| Terminal cost weight | `λ` | 1.0 | $/kWh² (equiv.) | Tune in analysis |
| Tikhonov weight | `ε` | 1e-3 | – | Small for conditioning |

**Action item:** The team should confirm or replace these with the actual data available. If real measurements exist from a lab microgrid, use them. If not, these are the values to use in the report and to generate synthetic disturbances (see §7).

---

## 7. Forecast Data (disturbances)

Provided as a single function `generate_forecasts.m` that returns a `24 × 3` matrix `[P_pv | P_load | c]`.

### 7.1 PV forecast `P_pv(k)`

Synthetic: clipped sinusoid peaking at solar noon.

```
P_pv(k) = P_pv_installed · max(0, sin(π · (k − 6) / 12))    for k = 6, …, 18
       = 0                                                   otherwise
```

Real-data alternative: NREL TMY data for Jakarta (−6.2° lat). Interpolation to hourly.

### 7.2 Load forecast `P_load(k)`

Synthetic: typical residential/commercial profile, two peaks (morning + evening).

```
P_load(k) = 0.5 · P_load_peak                                  (overnight, k = 0..5, 22..23)
         + 0.8 · P_load_peak + 0.2 · P_load_peak · sin(...)    (morning peak, k = 6..9)
         + 0.4 · P_load_peak                                   (midday, k = 10..16)
         + 0.9 · P_load_peak + 0.1 · P_load_peak · sin(...)    (evening peak, k = 17..21)
```

Real-data alternative: actual UI campus load profile if accessible.

### 7.3 Electricity price `c(k)`

TOU (time-of-use) tariff. Two options:

- **Indonesian PLN "WBP/LWBP" categories** (WBP = peak hours 18:00–22:00, LWBP = off-peak): c_WBP = 1.5 × c_base, c_LWBP = c_base. (Specific values per PLN regulation; team to confirm.)
- **Synthetic (for reproducibility):** c(k) = c_base for k = 0..17, 22..23; c(k) = 2.5 × c_base for k = 18..21.

c_base is a placeholder; team to confirm with current PLN rate. Suggest c_base = 1500 IDR/kWh ≈ $0.10/kWh as a starting reference (matches Cortes-Aguirre 2024's $0.10/kWh).

---

## 8. Simulation (not in your piece; document the contract so the simulation teammate can integrate)

The simulation teammate runs closed-loop over 7 days (or 24 h for the assignment plots):

```
For k = 0, 1, 2, … :
    1. Measure x(k)                  (from previous step or initial)
    2. Build QP: H, f, A_eq, b_eq, A_ineq, b_ineq, lb, ub
       using x(k) and forecast P_pv(k..k+Np-1), P_load(k..k+Np-1), c(k..k+Np-1)
    3. Solve QP: U* = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, lb, ub)
    4. Apply first control: u(k) = U*(1:2)
    5. Advance: x(k+1) = A·x(k) + B·u(k) + E·d(k)
    6. Record x(k), u(k), c(k) for plotting
    7. k ← k + 1
```

**Required plots** (per assignment example image):

1. Electricity price `c(k)` over 24 h
2. Battery power `u_batt(k)` over 24 h (positive = discharge)
3. SoC trajectory `x(k)` over 24 h or 7 days
4. Total cost `Σ c(k)·u_grid(k)·ΔT` cumulative over days
5. Grid import `u_grid(k)` over 24 h

**Baseline for comparison:** "No MPC" — always import `P_load − P_pv` directly, ignore battery. Quantify cost reduction (should be non-trivial per the literature; e.g. Cortes-Aguirre 2024 got 2% annual cost reduction with a more sophisticated model; ours with a simpler 1-day model and 0.5C battery should achieve more, but absolute numbers depend on price profile).

---

## 9. File Hand-off Package (in `/program/`, per the assignment's Section 7 folder structure)

This is what you commit to `Group7_MicrogridMPC/program/` (or wherever the team agrees). Each file is documented in a top-of-file comment block.

| File | Purpose | Inputs | Outputs |
|---|---|---|---|
| `plant_model.m` | Returns A, B, C, D, E, F_e matrices | (E_nom, ΔT) | A, B, C, D, E, F_e |
| `build_prediction_matrices.m` | Constructs Φ_x, Γ_x, Ψ_x | (Np, ΔT, E_nom) | Φ_x, Γ_x, Ψ_x |
| `build_constraints.m` | Constructs A_eq, b_eq, A_ineq, b_ineq, lb, ub | (Np, parameters, x(k), forecast) | constraint matrices |
| `build_qp.m` | Given (x(k), forecast), returns H, f | (x(k), forecast, parameters, λ, ε) | H, f |
| `parameters.m` | Returns all numerical parameters | none | struct with all §6 values |
| `generate_forecasts.m` | Returns 24×3 disturbance matrix | (date, P_pv_installed, P_load_peak, c_base) | D = [P_pv, P_load, c] |
| `README.md` | Interface contract for teammates | none | documentation |

**README.md** explicitly states:
- Variable names and units (kW, kWh, $/kWh, h, dimensionless)
- Sign convention (grid import positive, battery discharge positive)
- v1 limitations (perfect forecast, no PV curtailment, no export)
- v2 extensions roadmap
- MATLAB version tested
- Required toolboxes (Optimization Toolbox for `quadprog`; otherwise `qpOASES` or `OSQP` via YALMIP)

---

## 10. Risks, Edge Cases, and Open Items

### 10.1 Known risks (mitigations in parentheses)

- **Infeasible QP** (e.g. load exceeds PV + battery + grid max): add slack variables on SoC bounds with penalty `M · ε_slack` (large `M`). Cortes-Aguirre 2024 uses hard bounds and accepts occasional infeasibility; v1 can match this and log a warning.
- **Solver failure** (numerical issue with `quadprog`): return previous `u`, log warning, continue simulation. Document the failure rate in the report.
- **Forecast error** (v1 assumes perfect): explicitly stated in the report's "Limitations" section. v2: add ±10–20% error to forecasts, show that the closed-loop cost is robust (Polanco Vásquez 2023 showed robust to 12% error; our simpler model should also be).
- **PV overproduction** (when `P_pv > P_load + P_charge_max`): with v1's no-export and no-curtailment, this is infeasible. **Two v2 options:** (a) allow export, (b) add `P_curtailed` as a slack with zero cost.
- **Battery SoC hits limit** at the end of horizon: handled by the QP inequality constraints automatically.
- **Sign confusion** between the two `D`'s (one is the state-space D matrix, one is the disturbance stacked vector): README explicitly disambiguates with different names (`D_ss` for state-space, `D_stacked` for disturbance vector).

### 10.2 Open items for the team to confirm

- **Real data vs synthetic:** does the team have access to actual UI lab microgrid data? If yes, replace synthetic forecasts.
- **Parameter values:** §6 values are placeholders. Confirm with team.
- **PLN tariff specifics:** exact `c_WBP / c_LWBP` ratio and `c_base`. Look up current regulation.
- **Folder name:** the assignment says `/project_name/` with subdirs `/program /reports /presentation /references /images /data`. Group's project folder name to be decided (e.g. `Group7_MicrogridMPC`).
- **Reference style:** IEEE numbered `[1]`, `[2]`, … or author-year `(Cortes-Aguirre et al., 2024)`? Indonesian academic norms suggest IEEE. Confirm with lecturer's preference.

### 10.3 What I am NOT doing (and where to find it in the literature)

- **Stochastic MPC** (scenario-based, probability of constraint satisfaction): see Polanco Vásquez 2023 (they test 12% error) and Vasilj 2019 (ARMA+Copula scenarios). Out of v1 scope; "Bonus Challenge" item in assignment.
- **Distributed EMPC** (multiple networked microgrids with ADMM): see Jia 2026. Out of v1 scope; "Bonus Challenge" item.
- **Robust MPC** (worst-case disturbance): not in any of the 4 papers surveyed; available in Rawlings/Mayne/Diehl textbook. Out of v1 scope.
- **Nonlinear battery** (voltage curve, SoC-dependent efficiency): out of v1 scope; "Open Items" in Cortes-Aguirre 2024.
- **Kalman filter for state estimation:** v1 assumes `x(k)` is measured perfectly. v2: add Luenberger or Kalman filter on `x`. "Bonus Challenge" item in assignment.

---

## 11. Intellectual Lineage (for the report's "References" section)

To make the academic attribution explicit. Your lecturer will want to see that the design is grounded in literature, not invented.

| Design element | Primary source | Borrowed from | Why this source |
|---|---|---|---|
| 1-state SoC, 2-control form (Section 2) | Cortes-Aguirre et al. 2024 (arXiv 2412.10851) | – | Direct match to microgrid BESS formulation. Your FINPRO research. |
| Power-balance equality (Section 2.4) | Cortes-Aguirre et al. 2024 (their Eq. 6c) | – | Verbatim equation. |
| 100% efficiency in dynamics, loss in cost | Cortes-Aguirre et al. 2024 (§II-D) | – | Explicit justification: "preserves convexity." |
| Linear stage cost `c(k)·u_grid(k)·ΔT` (Section 4.1) | Cortes-Aguirre et al. 2024 (their Eq. 1) | Ellis et al. 2014, Rawlings et al. 2012 (canonical EMPC form) | The simplest EMPC form; standard across literature. |
| Terminal cost `λ·(x(N_p) − x_target)²` (Section 4.2) | Vasilj et al. 2019 (their `V_s`) AND Cortes-Aguirre et al. 2024 (their `V_f`, Eq. 12) | – | Simplified version of both. Vasilj calls it "cost-to-go of day-ahead layer," Cortes-Aguirre calls it "online reference trajectory." |
| Tikhonov regularization `ε·\|u\|²` (Section 4.3) | Standard MPC practice (Rawlings, Mayne, Diehl 2017, §2.6) | – | Well-conditioning; standard. |
| Prediction matrices F and Φ in stacked form (Section 3) | Rawlings, Mayne, Diehl 2017 (textbook, Chapter 2) | – | **Not in any of the 4 papers**; the assignment requires it. Textbook derivation. |
| Hourly ΔT, 24h horizon, perfect forecast (Sections 6, 7) | Cortes-Aguirre et al. 2024 (coarsened from 15 min to 1 h to match assignment plots) | – | Assignment's example plots are hourly. |
| Lab-scale parameters (Section 6) | **Not from any paper** — own estimates for UI lab scale, **flagged as placeholders** | – | Section 6 explicitly says "Pick the actual values with your team." |
| Two-layer day-ahead + real-time (v2, not v1) | Vasilj et al. 2019 | – | Single-layer simplification in v1. |
| Multi-objective cost + CO₂ (v2, not v1) | Polanco Vásquez et al. 2023 (their Pareto GA) | – | v1 is cost-only. |
| Distributed ADMM (v2, not v1) | Jia et al. 2026 | – | Different problem class (frequency/diesel, not energy dispatch). |
| Foundational EMPC definitions (stage cost, terminal cost, dissipativity) | Ellis, Durand, Christofides 2014 (J. Process Control, 663 citations) | Rawlings, Angeli, Bates 2012 (IEEE CDC) | Canonical references. |
| Standard MPC textbook (for F, Φ derivation, QP formulation) | Rawlings, Mayne, Diehl 2017 (Nob Hill) | – | The MPC textbook. |

---

## 12. References

In IEEE style (to be confirmed with lecturer; switch to author-year if preferred):

1. C. Cortes-Aguirre, Y.-A. Chen, A. Ghosh, J. Kleissl, and A. Khurram, "Economic MPC with an Online Reference Trajectory for Battery Scheduling Considering Demand Charge Management," arXiv preprint arXiv:2412.10851, Dec. 2024, submitted to *IEEE Trans. Smart Grid*.
2. J. Vasilj, S. Gros, D. Jakus, and M. Zanon, "Day-Ahead Scheduling and Real-Time Economic MPC of CHP Unit in Microgrid With Smart Buildings," *IEEE Trans. Smart Grid*, vol. 10, no. 2, pp. 1992–2001, Mar. 2019, doi: 10.1109/TSG.2017.2785500.
3. L. O. Polanco Vásquez, J. López Redondo, J. D. Álvarez Hervás, V. M. Ramírez, and J. L. Torres, "Balancing CO₂ emissions and economic cost in a microgrid through an energy management system using MPC and multi-objective optimization," *Applied Energy*, vol. 347, art. 120998, Sept. 2023, doi: 10.1016/j.apenergy.2023.120998.
4. Y. Jia, P. Yong, C. Li, K. Meng, Z. Y. Dong, and C. Sun, "An ADMM-Based Resilient Distributed Economic MPC Algorithm for Frequency Restoration and Economic Dispatch in Networked Microgrids," *IEEE Trans. Industry Applications*, vol. 62, no. 2, pp. 3275–3285, Mar./Apr. 2026, doi: 10.1109/TIA.2025.3618824.
5. M. Ellis, H. Durand, and P. D. Christofides, "A tutorial review of economic model predictive control methods," *J. Process Control*, vol. 24, no. 8, pp. 1156–1178, Aug. 2014, doi: 10.1016/j.jprocont.2014.03.010.
6. J. B. Rawlings, D. Angeli, and C. N. Bates, "Fundamentals of economic model predictive control," in *Proc. 51st IEEE Conf. Decision and Control (CDC)*, Maui, HI, USA, 2012, pp. 3851–3861.
7. J. B. Rawlings, D. Q. Mayne, and M. M. Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed. Nob Hill Publishing, 2017 (available free online).

---

## 13. Acceptance Criteria (how you'll know the modeling piece is done)

- [ ] `plant_model.m` returns the matrices in §2.2 and §2.3 with a sanity check (determinant, dimensions).
- [ ] `build_prediction_matrices.m` returns Φ_x, Γ_x, Ψ_x as specified in §3.
- [ ] `build_constraints.m` returns the constraint matrices in §5.
- [ ] `build_qp.m` returns a positive-definite H matrix and the correct f vector.
- [ ] `generate_forecasts.m` returns a 24×3 matrix with non-negative entries.
- [ ] `parameters.m` returns a struct with all §6 values.
- [ ] `README.md` documents sign convention, units, and v1 limitations.
- [ ] A 5-line sanity-check script runs end-to-end: load parameters, generate forecast, build QP for a sample `x(0)`, solve, print optimal `u(0)`. The output should be a finite real vector.
- [ ] Closed-loop simulation (run by simulation teammate) converges over 7 days without QP infeasibility.
- [ ] All 7 references cited in §12 are in the report's reference list.

---

## 14. Schedule (5 days remaining)

| Day | Date | Task | Deliverable |
|---|---|---|---|
| 1 | Fri 2026-06-05 | **Now.** Design approved, files scaffolded. | This spec committed. |
| 1 | Fri 2026-06-05 (PM) | Implement `parameters.m`, `plant_model.m`, `generate_forecasts.m`. | 3 .m files + README skeleton. |
| 2 | Sat 2026-06-06 | Implement `build_prediction_matrices.m`, `build_constraints.m`, `build_qp.m`. Sanity-check script. | 4 .m files + sanity-check passing. |
| 2 | Sat 2026-06-06 (PM) | Hand off to MPC-formulation teammate; integration test with their QP solver. | Joint working QP. |
| 3 | Sun 2026-06-07 | Industrial background write-up (Section 1 of the report structure). | 2-3 pages draft. |
| 3 | Sun 2026-06-07 (PM) | System description with parameters table (Section 2 of the report structure). | 1-2 pages draft. |
| 4 | Mon 2026-06-08 | State-space derivation write-up (Section 2 of report; this is what grades Pemodelan Sistem 20%). | 2-3 pages with equations. |
| 4 | Mon 2026-06-08 (PM) | Final report integration; presentation slides draft. | Report draft v1, slides v1. |
| 5 | Tue 2026-06-09 | Rehearsal, final review, edge case fixes. | Report v2, slides v2. |
| 5 | Tue 2026-06-09 (PM) | Submit via EMAS before 23:59 WIB. | Submitted. |

(Buffer day Wednesday 2026-06-10 is **deadline day**; submit by 18:00 to avoid last-minute issues.)

---

## 15. Open Questions for the Team

1. **Reference style:** IEEE numbered or author-year? (Affects report format, not this spec.)
2. **Project folder name:** `Group7_MicrogridMPC` or something else?
3. **Real data vs synthetic:** does the team have access to real UI microgrid data?
4. **PLN tariff values:** what are the current WBP/LWBP rates?
5. **MATLAB version:** is the Optimization Toolbox available? If not, fallback is YALMIP + OSQP (free, but slower to set up).
6. **Commit cadence:** push to a shared Git repo (GitHub/EMAS), or share via EMAS file upload?

---

**End of spec. Awaiting user approval before writing the GSD planning artifacts (PROJECT.md, REQUIREMENTS.md, ROADMAP.md, STATE.md, config.json) into `/home/wawabobo/MATLAB/SKPA FINPRO/.planning/`.**
