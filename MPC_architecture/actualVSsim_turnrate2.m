%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulated result using second order Nomoto compared with actual r
% Test with step in delta_c (commanded rudder angle)
% Remember to turn of disturbances

% Extract from simdata (same step window as before)
r     = simdata(:, 3);      % [rad/s]
delta = simdata(:, 7);      % [rad]

% Crop to the excited window, adjust indices to match step timing
idx_start = find(t >= 500,  1);
idx_end   = find(t >= 1400, 1);
idx = idx_start:idx_end;

r_win     = r(idx);
delta_win = delta(idx);
t_win     = t(idx) - t(idx(1));    % shift to start at zero

% Filter with a wider window than first order (double diff = more noise)
r_f    = sgolayfilt(r_win, 3, 51);

% Double differentiation
r_dot  = gradient(r_f,    h);
r_ddot = gradient(r_dot,  h);

% Trim boundary noise (gradient is unreliable at edges)
trim = 50;
r_t     = r_f(trim:end-trim);
rd_t    = r_dot(trim:end-trim);
rdd_t   = r_ddot(trim:end-trim);
delta_t = delta_win(trim:end-trim);
t_t     = t_win(trim:end-trim);

% Least-squares
Phi   = [-rd_t,  -r_t,  delta_t];  % [N x 3]
theta = Phi \ rdd_t;

a1 = theta(1);   % (T1+T2)/(T1*T2)
a0 = theta(2);   % 1/(T1*T2)
b  = theta(3);   % K/(T1*T2)

% Recover physical parameters
T1T2     = 1 / a0;
T1plusT2 = a1 * T1T2;
K_hat    = b  * T1T2;

% Quadratic: x^2 - (T1+T2)x + T1*T2 = 0
disc = T1plusT2^2 - 4*T1T2;

if disc < 0
    warning('Discriminant < 0: poles are complex, fall back to 1st order model')
else
    T1_hat = (T1plusT2 + sqrt(disc)) / 2;   % larger time constant
    T2_hat = (T1plusT2 - sqrt(disc)) / 2;   % smaller time constant
    fprintf('T1 = %.1f s,  T2 = %.1f s,  K = %.4f\n', T1_hat, T2_hat, K_hat)
    G2 = tf(K_hat, [T1_hat*T2_hat,  T1_hat+T2_hat,  1]);
    r_hat2 = lsim(G2, delta_t, t_t, r_t(1));

    figure;
    plot(t_t, r_t, 'b', t_t, r_hat2, 'r--', 'LineWidth', 1.5)
    legend('measured r', '2nd order Nomoto'); grid on
    ylabel('Yaw rate (rad/s)'); xlabel('Time (s)')
end


% A good 2nd order fit will track the S-shaped transient better than the 1st order did.
% If the two curves are nearly identical, the extra complexity of the 2nd order model
% isn't buying you anything and you should stay with the 1st order.
% T1 should be similar to T in first order, T2 should be faster/smaller
