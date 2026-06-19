clc; clear; close all;

with = load('simdata_CBF_consecutive_w_d.mat');   % loads simdata and t
without = load('simdata_CBF_consecutive_wo_d.mat');   % loads simdata and t

t_without = without.t;
t_with    = with.t;

% --- without ---
u_without           = without.simdata(:,1);                 % m/s
v_without           = without.simdata(:,2);                 % m/s
r_without           = without.simdata(:,3);                 % rad/s
r_deg_without       = (180/pi) * r_without;                 % deg/s
x_without           = without.simdata(:,4);                 % m
y_without           = without.simdata(:,5);                 % m
psi_without         = without.simdata(:,6);                 % rad
psi_deg_without     = (180/pi) * psi_without;               % deg
delta_deg_without   = (180/pi) * without.simdata(:,7);       % deg
n_without           = without.simdata(:,8);                  % rpm
n_c_without         = without.simdata(:,10);                 % rpm
delta_c_deg_without = (180/pi) * without.simdata(:,9);        % deg
u_d_without         = without.simdata(:,11);                 % m/s
psi_d_without       = without.simdata(:,12);                 % rad
psi_d_deg_without   = (180/pi) * psi_d_without;              % deg
r_d_without         = without.simdata(:,13);                 % rad/s
r_d_deg_without     = (180/pi) * r_d_without;                % deg/s
beta_without        = without.simdata(:,14);                 % rad
beta_deg_without    = (180/pi) * beta_without;               % deg
beta_c_without      = without.simdata(:,15);                 % rad
beta_c_deg_without  = (180/pi) * beta_c_without;             % deg
x_p_without         = without.simdata(:,16);                 % m
y_p_without         = without.simdata(:,17);                 % m
delta_without       = without.simdata(:,18);                 % m
psi_ref_without     = without.simdata(:,19);                 % rad
psi_ref_deg_without = (180/pi) * psi_ref_without;            % deg
y_e_max_without     = without.simdata(:,20);                 % m
dx_p_without        = without.simdata(:,21);
dy_p_without        = without.simdata(:,22);
y_e_without         = without.simdata(:,23);                 % m
psi_d_safe_without      = without.simdata(:,24);             % rad
psi_d_safe_deg_without  = (180/pi) * psi_d_safe_without;     % deg
cbf_val_without         = without.simdata(:,25);

% --- with ---
u_with           = with.simdata(:,1);                 % m/s
v_with           = with.simdata(:,2);                 % m/s
r_with           = with.simdata(:,3);                 % rad/s
r_deg_with       = (180/pi) * r_with;                 % deg/s
x_with           = with.simdata(:,4);                 % m
y_with           = with.simdata(:,5);                 % m
psi_with         = with.simdata(:,6);                 % rad
psi_deg_with     = (180/pi) * psi_with;               % deg
delta_deg_with   = (180/pi) * with.simdata(:,7);       % deg
n_with           = with.simdata(:,8);                  % rpm
n_c_with         = with.simdata(:,10);                 % rpm
delta_c_deg_with = (180/pi) * with.simdata(:,9);        % deg
u_d_with         = with.simdata(:,11);                 % m/s
psi_d_with       = with.simdata(:,12);                 % rad
psi_d_deg_with   = (180/pi) * psi_d_with;              % deg
r_d_with         = with.simdata(:,13);                 % rad/s
r_d_deg_with     = (180/pi) * r_d_with;                % deg/s
beta_with        = with.simdata(:,14);                 % rad
beta_deg_with    = (180/pi) * beta_with;               % deg
beta_c_with      = with.simdata(:,15);                 % rad
beta_c_deg_with  = (180/pi) * beta_c_with;             % deg
x_p_with         = with.simdata(:,16);                 % m
y_p_with         = with.simdata(:,17);                 % m
delta_with       = with.simdata(:,18);                 % m
psi_ref_with     = with.simdata(:,19);                 % rad
psi_ref_deg_with = (180/pi) * psi_ref_with;            % deg
y_e_max_with     = with.simdata(:,20);                 % m
dx_p_with        = with.simdata(:,21);
dy_p_with        = with.simdata(:,22);
y_e_with         = with.simdata(:,23);                 % m
psi_d_safe_with      = with.simdata(:,24);             % rad
psi_d_safe_deg_with  = (180/pi) * psi_d_safe_with;     % deg
cbf_val_with         = with.simdata(:,25);

% corridor boundaries
x_right = x_p_with - y_e_max_with .* dy_p_with;
y_right = y_p_with + y_e_max_with .* dx_p_with;
x_left = x_p_with + y_e_max_with .* dy_p_with;
y_left = y_p_with - y_e_max_with .* dx_p_with;

figure(1); hold on; axis equal; grid on;
fill([y_left; flipud(y_right)], [x_left; flipud(x_right)], ...
    [0.8 0.9 1.0], 'EdgeColor','none', 'DisplayName','Corridor');
plot(y_p_with, x_p_with, 'r-', 'LineWidth',1.5, 'DisplayName','Path')
plot(y_with, x_with, 'b--', 'LineWidth',1.5, 'DisplayName','with V_c=0.8m/s')
plot(y_without, x_without, 'g--', 'LineWidth',1.5, 'DisplayName','with V_c=0.0m/s')
xlabel('East [m]'); ylabel('North [m]');    % Remember x,y is opposide because of NED
title('Path tracking with corridor');
legend;

figure(2); hold on; grid on;
fill([t_with'; flipud(t_with')], [y_e_max_with; flipud(-y_e_max_with)], [0.8 0.9 1.0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.3, 'DisplayName', 'Corridor');
xlabel('Time (s)'); ylabel('Cross-track error (m)');
plot(t_with, y_e_with, 'b', 'LineWidth', 1.5, 'DisplayName', 'Cross-track error y_e with V_c=0.8m/s');
plot(t_without, y_e_without, '-', 'LineWidth', 1.5, 'DisplayName', 'Cross-track error y_e with V_c=0.0m/s');
plot(t_with, y_e_max_with, 'r--', 'LineWidth', 1.5, 'DisplayName', 'y_{e,max}');
plot(t_with, -y_e_max_with, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
title('Cross track error y_e');
legend; ylim([-1.2*max(y_e_max_with), 1.2*max(y_e_max_with)]);

figure(3);
subplot(211)
plot(t_with, psi_d_safe_deg_with,'-', t_without, psi_d_safe_deg_without,'-', 'linewidth', 2);
title('\psi_{d,safe} with vs without disturbance'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\psi_{d,safe} with V_c=0.8m/s', '\psi_{d,safe} with V_c=0.0/s')
grid on;
subplot(212)
plot(t_with,delta_c_deg_with, t_without,delta_c_deg_without, '-','linewidth',2);
title('Commanded rudder angle with and without disturbance'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\delta_c with V_c=0.8m/s','\delta_c with V_c=0.0m/s')
grid on;