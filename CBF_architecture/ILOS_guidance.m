function [psi_ref,y_int_dot] = ILOS_guidance(e_y,pi_p,y_int,Delta, delta_c)
kappa0 = 2.5;
L = 161;

% Adaptive kappa
kappa = kappa0 * exp(-abs(e_y)/L);

Kp = 1/(Delta*1);
Ki = kappa * Kp;

% Integrator bounding
y_int_max = 30;
y_int = min(max(y_int, -y_int_max), y_int_max);

% Desired heading
psi_ref = pi_p - atan2(Kp * e_y + Ki * y_int, 1);

% Anti windup
delta_max = 40 * pi/180;
% 0.95 * delta_max because delta_c is already getting clamped in PID/FBL
% controller, so this should insure the antiwindup actually kicks in
if abs(delta_c) >= 0.95 * delta_max && sign(delta_c) == sign(e_y)
    y_int_dot = 0 * y_int;
else
    % Integrator dynamics
    y_int_dot = Delta * e_y / (Delta^2 + (e_y + kappa * y_int)^2);
end

end