function xd_dot = ref_model(xd, psi_ref)

    omega_n = 0.15; 
    % zeta = 1  (implicit in this "triple pole" implementation)

    % xd = [psi_d; r_d; rdot_d], psi_ref in radians
    psi_d  = xd(1);
    r_d    = xd(2);
    rdot_d = xd(3);

    % Shortest-path angular error to avoid 2π jumps
    e = wrapToPi(psi_ref - psi_d);

    % (s + omega)^3 structure written as 3 cascaded first order blocks:
    psi_d_dot  = r_d;
    r_d_dot    = rdot_d;
    rdot_d_dot = omega_n^3 * e - 3*omega_n^2 * r_d - 3*omega_n * rdot_d;

    xd_dot = [psi_d_dot; r_d_dot; rdot_d_dot];
end
