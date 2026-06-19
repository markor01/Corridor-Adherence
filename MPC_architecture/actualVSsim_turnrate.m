%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulated result using first order Nomoto compared with actual r
% Test with step in delta_c (commanded rudder angle)
% Remember to turn of disturbances

% After running the step test, extract from simdata:
r     = simdata(:, 3);          % yaw rate [rad/s]
delta = simdata(:, 7);          % actual rudder [rad]

% Numerical derivative (filter first to reduce noise)
r_f   = sgolayfilt(r, 3, 11);  % Savitzky-Golay, order 3, 11-sample window
r_dot = gradient(r_f, h);

% Regression matrix and solve
Phi   = [-r_f, delta];
theta = Phi \ r_dot;

T_hat =  1 / theta(1);
K_hat =  theta(2) * T_hat;

fprintf('T = %.1f s,  K = %.4f', T_hat, K_hat)
assert(T_hat > 0, 'T must be positive — check regression sign')
assert(K_hat > 0, 'K must be positive for a right-turning rudder')

r_hat = lsim(tf(K_hat, [T_hat 1]), delta, t, r(1));
plot(t, r_deg, t, rad2deg(r_hat), 'LineWidth',2);
grid;
xlabel('Time (s)'); ylabel('deg/s');
title('First order Nomoto model vs actual turn rate response')
% legend('measured r', 'Nomoto model');
legend('r(t)', 'Nomoto model');

%If the curves match well, T and K are good. If the model is too slow/fast, adjust T.