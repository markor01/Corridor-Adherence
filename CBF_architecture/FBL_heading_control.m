function delta_c = FBL_heading_control(x, psi_d, r_d, r_d_dot, i)
% Feedback linearization controller using Nomoto model and pole placement

% Nomoto model: T*r_dot + r = K*delta

%               r_dot = -1/T * r + K/T * delta

% Choose delta s.t. r_dot = v (virtual control)
    
    psi = x(6);
    r = x(3);

    % Nomoto parameters (to be tuned or identified)
%     T = 50;     % time constant  Time for r=0.63*r_ss
%     K = 0.027;    % rudder gain K=r_ss/delta_step=0.267/10
    T = 50;
    K = 0.03;

    % Errors
    e_psi = ssa(psi - psi_d);
    e_r = r - r_d;
    
    % Desired yaw acceleration for virtual control v
    wn = 0.20;  % natural freq, rule of thumb: 5/T, I find that 0.15 gives a bit less oscillations
    zeta = 1;   % damping

    k1 = wn^2;
    k2 = 2 * zeta * wn;

    v = -k1*e_psi - k2*e_r + r_d_dot;
    
    % delta that cancels out the Nomoto model
    alpha = 1;    % alpha = 1 is the normal FBL case
    delta_unsat = T/K * (v + alpha * 1/T * r); 

    % --- Rudder saturation ---
    delta_max = 40*pi/180;                            % [rad]
    delta_c = max(-delta_max, min(delta_max, delta_unsat));

   
end

