function [delta_c, e_int_out] = PID_heading(e_psi, e_r, e_int_in, h)
% PID heading controller based on Fossen (2021) Algorithm 15.1
%
% Inputs:
%   e_psi     - heading error [rad], wrapped to [-pi, pi]
%   e_r       - yaw rate error [rad/s]
%   e_int_in  - integral of heading error from previous step
%   h         - sample time [s]
%
% Outputs:
%   delta_c   - commanded rudder angle [rad]
%   e_int_out - updated integral state

    % Nomoto parameters (from step test identification)
    T = 50;       % time constant [s]
    K = 0.027;   % rudder gain [rad/s per rad]

    % Desired closed loop behaviour
    omega_b = 0.1;   % bandwidth [rad/s]
    zeta    = 1.0;    % damping ratio

    % PID gains (Fossen 2021, Algorithm 15.1)
    Kp = (omega_b^2 * T) / K;
    Kd = (2 * zeta * omega_b * T - 1) / K;
    Ki = Kp * omega_b;

    % Control law
    delta_unsat = -(Kp * e_psi + Kd * e_r + Ki * e_int_in);

    % Rudder saturation
    delta_max = 40 * pi/180;   % [rad]
    delta_c   = max(-delta_max, min(delta_max, delta_unsat));

    % Anti-windup: conditional integration
    sat_err = delta_unsat - delta_c;
    if abs(sat_err) < 1e-12 || sign(sat_err) ~= sign(Ki * e_psi)
        e_int_out = e_int_in + h * e_psi;
    else
        e_int_out = e_int_in;
    end
end