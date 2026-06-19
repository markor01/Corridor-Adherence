clc; clear; close all;

with = load('simdata_MPC_disturbance20.mat');   % loads simdata and t
without = load('simdata_MPC_disturbance08.mat');   % loads simdata and t

t_without = without.t;
t_with    = with.t;

% --- without ---
u_without                = without.simdata(:,1);                 % m/s
v_without                = without.simdata(:,2);                 % m/s
r_without                = without.simdata(:,3);                 % rad/s
r_deg_without            = (180/pi) * r_without;                 % deg/s
x_without                = without.simdata(:,4);                 % m
y_without                = without.simdata(:,5);                 % m
psi_without              = without.simdata(:,6);                 % rad
psi_deg_without          = (180/pi) * psi_without;               % deg
delta_deg_without        = (180/pi) * without.simdata(:,7);      % deg
n_without                = without.simdata(:,8);                 % rpm
n_c_without              = without.simdata(:,10);                % rpm
delta_c_deg_without      = (180/pi) * without.simdata(:,9);      % deg
u_d_without              = without.simdata(:,11);                % m/s
pi_p_without             = without.simdata(:,12);                % rad
pi_p_deg_without         = (180/pi) * pi_p_without;              % deg
x_p_without              = without.simdata(:,13);                % m
y_p_without              = without.simdata(:,14);                % m
y_e_max_without          = without.simdata(:,15);                % m
dx_p_without             = without.simdata(:,16);
dy_p_without             = without.simdata(:,17);
y_e_without              = without.simdata(:,18);                % m
infeasible_count_without = without.simdata(:,19);

% --- with ---
u_with                = with.simdata(:,1);                 % m/s
v_with                = with.simdata(:,2);                 % m/s
r_with                = with.simdata(:,3);                 % rad/s
r_deg_with            = (180/pi) * r_with;                 % deg/s
x_with                = with.simdata(:,4);                 % m
y_with                = with.simdata(:,5);                 % m
psi_with              = with.simdata(:,6);                 % rad
psi_deg_with          = (180/pi) * psi_with;               % deg
delta_deg_with        = (180/pi) * with.simdata(:,7);      % deg
n_with                = with.simdata(:,8);                 % rpm
n_c_with              = with.simdata(:,10);                % rpm
delta_c_deg_with      = (180/pi) * with.simdata(:,9);      % deg
u_d_with              = with.simdata(:,11);                % m/s
pi_p_with             = with.simdata(:,12);                % rad
pi_p_deg_with         = (180/pi) * pi_p_with;              % deg
x_p_with              = with.simdata(:,13);                % m
y_p_with              = with.simdata(:,14);                % m
y_e_max_with          = with.simdata(:,15);                % m
dx_p_with             = with.simdata(:,16);
dy_p_with             = with.simdata(:,17);
y_e_with              = with.simdata(:,18);                % m
infeasible_count_with = with.simdata(:,19);

% corridor boundaries
x_right = x_p_with - y_e_max_with .* dy_p_with;
y_right = y_p_with + y_e_max_with .* dx_p_with;
x_left = x_p_with + y_e_max_with .* dy_p_with;
y_left = y_p_with - y_e_max_with .* dx_p_with;

figure(1); hold on; axis equal; grid on;
fill([y_left; flipud(y_right)], [x_left; flipud(x_right)], ...
    [0.8 0.9 1.0], 'EdgeColor','none', 'DisplayName','Corridor');
plot(y_p_with, x_p_with, 'r-', 'LineWidth',1.5, 'DisplayName','Path')
plot(y_with, x_with, 'b--', 'LineWidth',1.5, 'DisplayName','with V_c=2.0m/s')
plot(y_without, x_without, 'g--', 'LineWidth',1.5, 'DisplayName','with V_c=0.8m/s')
xlabel('East [m]'); ylabel('North [m]');    % Remember x,y is opposide because of NED
title('Path tracking with corridor');
legend;

figure(2); hold on; grid on;
fill([t_with'; flipud(t_with')], [y_e_max_with; flipud(-y_e_max_with)], [0.8 0.9 1.0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.3, 'DisplayName', 'Corridor');
xlabel('Time (s)'); ylabel('Cross-track error (m)');
plot(t_with, y_e_with, 'b', 'LineWidth', 1.5, 'DisplayName', 'Cross-track error y_e with V_c=2.0m/s');
plot(t_without, y_e_without, '-', 'LineWidth', 1.5, 'DisplayName', 'Cross-track error y_e with V_c=0.8m/s');
plot(t_with, y_e_max_with, 'r--', 'LineWidth', 1.5, 'DisplayName', 'y_{e,max}');
plot(t_with, -y_e_max_with, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
title('Cross track error y_e');
legend; ylim([-1.2*max(y_e_max_with), 1.2*max(y_e_max_with)]);

figure(3);
subplot(211)
plot(t_with,delta_c_deg_with, t_without,delta_c_deg_without, '-','linewidth',2);
title('Commanded rudder angle with and without disturbance'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\delta_c with V_c=2.0m/s','\delta_c with V_c=0.8m/s')
grid on;
subplot(212)
plot(t_with, infeasible_count_with, t_without, infeasible_count_without, '-', 'LineWidth',1.5);
title('Infeasible count with and without disturbance'); xlabel('Time (s)'); ylabel('Consecutive infeasible QPs');
legend('V_c=2.0m/s','V_c=0.8m/s')
grid on;
