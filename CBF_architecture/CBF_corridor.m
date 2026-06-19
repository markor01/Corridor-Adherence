function [psi_d_safe, cbf_val] = CBF_corridor(psi_d, e_y, e_y_max, pi_p, U, alpha)
% Guidance level control barrier function for corridor guarantee

% Filters desired heading psi_d to ensure the ship remains within corridor.
% Applied before heading controller and after reference model

% Architecture:
%   ILOS -> psi_ref -> ref_model -> psi_d -> CBF_corridor -> psi_d_safe
%       -> heading controller (PID/FBL)

% Inputs:
%   psi_d       - desired heading from ref_model [rad]
%   e_y         - cross-track error [m] (positive value -> right side of path)
%   e_y_max     - corridor half width [m]
%   pi_p        - path tangential angle [rad]
%   U           - ship total speed [m/s]
%   alpha       - CBF gain

% Outputs
%   psi_d_safe  - modified desired heading satisfying the CBF constraint

% Guard near zero speed
if abs(U) < 0.1
    psi_d_safe = psi_d;
    return;
end

% Heading relative to path tangengtial angle
psi_rel = ssa(psi_d - pi_p);

% Predict e_y assuming current drift rate persists over t_horizon
% e_y_dot = U*sin(psi-pi_p) is the sideways drift rate relative to path
% Works for both straight and curved segments since e_y is always
% perpendicular to path
t_horizon = 80;     % Accounts for Nomoto lag ~50s (Tuning parameter)
e_y_dot = U * sin(psi_rel);
e_y_pred = e_y + e_y_dot * t_horizon;

if sign(e_y_pred) ~= sign(e_y)
    psi_d_safe = psi_d;
    cbf_val = 0;
    return;
end

% Barrier evaluated at predicted position
h = e_y_max^2 - e_y_pred^2;

% CBF condition at predicted state
% cbf_val = h_dot + alpha * h
% cbf_val = -2 * e_y * U * sin(psi_rel) + alpha * h;
cbf_val = -2 * e_y_pred * U * sin(psi_rel) + alpha * h;

% if enough margin to boundary
if cbf_val >= 0
    % No modification necessary
    psi_d_safe = psi_d;
    return;
end

% Compute safe heading at the corridor boundary
% sin_lim is the boundary value of sin(psi - pi_p) and it
% represents how much the ship is allowed to be heading into the boundary
% Solve: -2 * e_y * U * sin(psi_rel_safe) + alpha * h = 0
%   => sin(psi_rel_safe) = (alpha * h) / (2 * e_y * U)
sin_lim = (alpha * h) / (2 * e_y_pred * U);
sin_lim = max(-1, min(1, sin_lim));     % clamp for numerical safety

psi_rel_safe = asin(sin_lim);
psi_d_cbf = ssa(pi_p + psi_rel_safe);

% Never move psi_d by more than max_delta
max_delta = deg2rad(20);
correction = ssa(psi_d_cbf - psi_d);
correction = max(-max_delta, min(max_delta, correction));
psi_d_safe = ssa(psi_d + correction);

end

