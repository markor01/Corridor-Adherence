clc; clear; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load CBF and MPC datasets
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% cbf_raw = load('simdata_CBF_nominal.mat');   % loads simdata and t
% mpc_raw = load('simdata_MPC_nominal.mat');   % loads simdata and t

% cbf_raw = load('simdata_CBF_disturbance_with.mat');   % loads simdata and t
% mpc_raw = load('simdata_MPC_disturbance08.mat');   % loads simdata and t

cbf_raw = load('simdata_CBF_consecutive_w_d.mat');   % loads simdata and t
mpc_raw = load('simdata_MPC_consecutive_w_d15.mat');   % loads simdata and t

t_cbf = cbf_raw.t;
t_mpc = mpc_raw.t;

sd_cbf = cbf_raw.simdata;   % 25 columns
sd_mpc = mpc_raw.simdata;   % 19 columns

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- CBF extraction ---
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u_cbf           = sd_cbf(:,1);                 % m/s
v_cbf           = sd_cbf(:,2);                 % m/s
r_cbf           = sd_cbf(:,3);                 % rad/s
r_deg_cbf       = (180/pi) * r_cbf;            % deg/s
x_cbf           = sd_cbf(:,4);                 % m
y_cbf           = sd_cbf(:,5);                 % m
psi_cbf         = sd_cbf(:,6);                 % rad
psi_deg_cbf     = (180/pi) * psi_cbf;          % deg
delta_deg_cbf   = (180/pi) * sd_cbf(:,7);      % deg
n_cbf           = sd_cbf(:,8);                 % rpm
delta_c_deg_cbf = (180/pi) * sd_cbf(:,9);      % deg
n_c_cbf         = sd_cbf(:,10);                % rpm
u_d_cbf         = sd_cbf(:,11);                % m/s
psi_d_cbf       = sd_cbf(:,12);                % rad
r_d_cbf         = sd_cbf(:,13);                % rad/s
beta_cbf        = sd_cbf(:,14);                % rad
beta_c_cbf      = sd_cbf(:,15);                % rad
x_p_cbf         = sd_cbf(:,16);                % m
y_p_cbf         = sd_cbf(:,17);                % m
lookahead_cbf   = sd_cbf(:,18);                % m
psi_ref_cbf     = sd_cbf(:,19);                % rad
y_e_max_cbf     = sd_cbf(:,20);                % m
dx_p_cbf        = sd_cbf(:,21);
dy_p_cbf        = sd_cbf(:,22);
y_e_cbf         = sd_cbf(:,23);                % m
psi_d_safe_cbf  = sd_cbf(:,24);                % rad
cbf_val_cbf     = sd_cbf(:,25);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- MPC extraction ---
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u_mpc                = sd_mpc(:,1);                 % m/s
v_mpc                = sd_mpc(:,2);                 % m/s
r_mpc                = sd_mpc(:,3);                 % rad/s
r_deg_mpc            = (180/pi) * r_mpc;            % deg/s
x_mpc                = sd_mpc(:,4);                 % m
y_mpc                = sd_mpc(:,5);                 % m
psi_mpc              = sd_mpc(:,6);                 % rad
psi_deg_mpc          = (180/pi) * psi_mpc;          % deg
delta_deg_mpc        = (180/pi) * sd_mpc(:,7);      % deg
n_mpc                = sd_mpc(:,8);                 % rpm
delta_c_deg_mpc      = (180/pi) * sd_mpc(:,9);      % deg
n_c_mpc              = sd_mpc(:,10);                % rpm
u_d_mpc              = sd_mpc(:,11);                % m/s
pi_p_mpc             = sd_mpc(:,12);                % rad
x_p_mpc              = sd_mpc(:,13);                % m
y_p_mpc              = sd_mpc(:,14);                % m
y_e_max_mpc          = sd_mpc(:,15);                % m
dx_p_mpc             = sd_mpc(:,16);
dy_p_mpc             = sd_mpc(:,17);
y_e_mpc              = sd_mpc(:,18);                % m
infeasible_count_mpc = sd_mpc(:,19);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Corridor boundaries (assumes both runs share the same path/corridor)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_right = x_p_cbf - y_e_max_cbf .* dy_p_cbf;
y_right = y_p_cbf + y_e_max_cbf .* dx_p_cbf;
x_left  = x_p_cbf + y_e_max_cbf .* dy_p_cbf;
y_left  = y_p_cbf - y_e_max_cbf .* dx_p_cbf;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figure 1: Path tracking with corridor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1); hold on; axis equal; grid on;
fill([y_left; flipud(y_right)], [x_left; flipud(x_right)], ...
    [0.8 0.9 1.0], 'EdgeColor','none', 'DisplayName','Corridor');
plot(y_p_cbf, x_p_cbf, 'r-', 'LineWidth',1.5, 'DisplayName','Path')
plot(y_cbf, x_cbf, 'b--', 'LineWidth',1.5, 'DisplayName','CBF architecture')
plot(y_mpc, x_mpc, 'm--',  'LineWidth',1.5, 'DisplayName','MPC architecture')
xlabel('East [m]'); ylabel('North [m]');    % Remember x,y is opposite because of NED
title('Path tracking with corridor');
legend;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figure 2: Cross-track error
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2); hold on; grid on;
fill([t_cbf'; flipud(t_cbf')], [y_e_max_cbf; flipud(-y_e_max_cbf)], [0.8 0.9 1.0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.3, 'DisplayName', 'Corridor');
plot(t_cbf, y_e_cbf, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Cross-track error y_e, CBF (V_c=0.8m/s)');
plot(t_mpc, y_e_mpc, 'm-',  'LineWidth', 1.5, 'DisplayName', 'Cross-track error y_e, MPC (V_c=1.5m/s)');
plot(t_cbf, y_e_max_cbf, 'r--', 'LineWidth', 1.5, 'DisplayName', 'y_{e,max}');
plot(t_cbf, -y_e_max_cbf, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Cross-track error (m)');
title('Cross track error y_e');
legend; ylim([-1.2*max(y_e_max_cbf), 1.2*max(y_e_max_cbf)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figure 3: Commanded rudder angle (both) and infeasible count (MPC only)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3);
plot(t_cbf, delta_c_deg_cbf, 'b-', t_mpc, delta_c_deg_mpc, 'm-', 'LineWidth',1.5);
title('Commanded rudder angle, CBF vs MPC'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\delta_c, CBF', '\delta_c, MPC')
grid on;