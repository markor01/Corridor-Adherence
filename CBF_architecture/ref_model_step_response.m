clear all; close all
xd = [0 0 0]';
psi_ref = [zeros(500,1); ones(1500,1)];
xd_data = zeros(2000,1);
h = 0.1;

for i = 1:2000

    omega_n = 0.15;
    % zeta = 1  (implicit in this "triple pole" implementation)

    
    % % xd = [psi_d; r_d; rdot_d], psi_ref in radians
    psi_d  = xd(1);
    r_d    = xd(2);
    rdot_d = xd(3);

    % Shortest-path angular error to avoid 2π jumps
    e = wrapToPi(psi_ref(i) - psi_d);

    % (s + omega)^3 structure written as 3 cascaded first order blocks:
    psi_d_dot  = r_d;
    r_d_dot    = rdot_d;
    rdot_d_dot = omega_n^3 * e - 3*omega_n^2 * r_d - 3*omega_n * rdot_d;

    xd_dot = [psi_d_dot; r_d_dot; rdot_d_dot];

    xd     = xd + h*xd_dot;     % Euler is fine
    psi_d  = xd(1);
    r_d    = xd(2);

%     psi_d  = psi_ref(i);
%     xd(1) = psi_d;
    xd_data(i) = xd(1);
    xd_dot_data(i) = xd(2);
    xd_ddot_data(i) = xd(3);

end

figure(1);
plot((1:2000)*h, psi_ref, 'k--', 'LineWidth', 1.5); hold on;
plot((1:2000)*h, xd_data, 'b', 'LineWidth', 1.5);
title('Reference model step response');
xlabel('Time (s)'); ylabel('rad');
ylim([-0.1,1.1]);
legend('psi\_ref','psi\_d');
grid on;