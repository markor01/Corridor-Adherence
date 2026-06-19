function n_c = speed_controller(u, u_d, e_int_u)
% PI closed loop speed controller

% Inputs:
%   u       - actual surge velocity [m/s]
%   u_d     - desired surge velocity [m/s]
%   e_int_u -intergral of surge error from prev step
%   h       - sample time [s]

% Outputs:
%   n_c     - commanded shaft velocity [rpm]

    % Feedforward, steady state rpm for desired speed
    t_thr = 0.05;                          % same as in ship.m
    Xu    = -(17.0677e6-(-8.9830e5))/20;   % same Xu as in ship.m (linearization)
    rho   = 1025;  D = 3.3;  KT = 0.6367;  % (J=0 Wageningen)                     
    Td_ff = -Xu/(1 - t_thr) * u_d;         % desired thrust [N]
    n_ff = sign(Td_ff) * sqrt(abs(Td_ff) / (rho*D^4*KT));   % rps
    n_ff   = 60 * n_ff;                                     % rpm

    % PI feedback
    Kp = 25;
    Ki = 0.2;

    e_u = u_d - u;
    n_c = n_ff + Kp * e_u + Ki * e_int_u;

end

