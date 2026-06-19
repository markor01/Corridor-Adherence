clc; clear; close all;

% load('simdata_MPC_nominal.mat');   % loads simdata and t

% load('simdata_MPC_R1250.mat');   % loads simdata and t

% load('simdata_MPC_R1250_narrow.mat');   % loads simdata and t

% load('simdata_MPC_disturbance08.mat');   % loads simdata and t

load('simdata_MPC_consecutive_w_d20.mat');   % loads simdata and t

u           = simdata(:,1);                 % m/s
v           = simdata(:,2);                 % m/s
r           = simdata(:,3);                 % rad/s
r_deg       = (180/pi) * r;                 % deg/s
x           = simdata(:,4);                 % m
y           = simdata(:,5);                 % m
psi         = simdata(:,6);                 % rad
psi_deg     = (180/pi) * psi;               % deg
delta_deg   = (180/pi) * simdata(:,7);      % deg
n   = simdata(:,8);    % rpm (state 8 is n in rpm)
n_c = simdata(:,10);   % rpm (command you stored)
delta_c_deg = (180/pi) * simdata(:,9);      % deg
u_d         = simdata(:,11);                % m/s
pi_p        = simdata(:,12);                % rad
pi_p_deg    = (180/pi) * pi_p;             % deg
x_p         = simdata(:,13);                % m
y_p         = simdata(:,14);                % m
e_y_max     = simdata(:,15);                % m
dx_p        = simdata(:,16);                
dy_p        = simdata(:,17);
e_y         = simdata(:,18);                % m
infeasible_count         = simdata(:,19);

% corridor boundaries
x_right = x_p - e_y_max .* dy_p;
y_right = y_p + e_y_max .* dx_p;
x_left = x_p + e_y_max .* dy_p;
y_left = y_p - e_y_max .* dx_p;

figure(1); hold on; axis equal; grid on;
fill([y_left; flipud(y_right)], [x_left; flipud(x_right)], ...
    [0.8 0.9 1.0], 'EdgeColor','none', 'DisplayName','Corridor');
plot(y_p, x_p, 'r-', 'LineWidth',1.5, 'DisplayName','Path')
plot(y, x, 'b--', 'LineWidth',1.5, 'DisplayName','Actual')
xlabel('East [m]'); ylabel('North [m]');    % Remember x,y is opposide because of NED
title('Path tracking with corridor');
legend;

figure(2); hold on; grid on;
fill([t'; flipud(t')], [e_y_max; flipud(-e_y_max)], [0.8 0.9 1.0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.3, 'DisplayName', 'Corridor');
xlabel('Time (s)'); ylabel('Cross-track error (m)');
plot(t, e_y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Cross-track error y_e');
plot(t, e_y_max, 'r--', 'LineWidth', 1.5, 'DisplayName', 'y_{e,max}');
plot(t, -e_y_max, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
title('Cross track error y_e');
legend; ylim([-1.2*max(e_y_max), 1.2*max(e_y_max)]);

figure(3)
plot(t,delta_deg,t,delta_c_deg,'linewidth',2);
title('Actual and commanded rudder angle'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\delta','\delta_c')
grid on;

figure(4); hold on; grid on;
plot(t, infeasible_count, 'b', 'LineWidth',1.5);
title('Infeasible count'); xlabel('Time (s)'); ylabel('Consecutive infeasible QPs');
