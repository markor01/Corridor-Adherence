clc; clear; close all;

load('simdata_CBF_nocorr_tight.mat');   % loads simdata and t

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
psi_d       = simdata(:,12);                % rad
psi_d_deg   = (180/pi) * psi_d;             % deg
r_d         =  simdata(:,13);               % rad/s
r_d_deg     = (180/pi) * r_d;               % deg/s
beta        = simdata(:,14);                % rad
beta_deg    = (180/pi) * beta;              % deg
beta_c      = simdata(:,15);                % rad
beta_c_deg  = (180/pi) * beta_c;            % deg
x_p         = simdata(:,16);                % m
y_p         = simdata(:,17);                % m
delta       = simdata(:,18);                % m
psi_ref     = simdata(:,19);                % rad
psi_ref_deg = psi_ref * (180/pi);           % deg
y_e_max     = simdata(:,20);                % m
dx_p        = simdata(:,21);                
dy_p        = simdata(:,22);
y_e         = simdata(:,23);                % m
psi_d_safe  = simdata(:,24);                % rad
psi_d_safe_deg   = (180/pi) * psi_d_safe;   % deg
cbf_val     = simdata(:,25);


figure(1); hold on; axis equal; grid on;
plot(y_p, x_p, 'r-', 'LineWidth',1.5, 'DisplayName','Path')
plot(y, x, 'b--', 'LineWidth',1.5, 'DisplayName','Actual')
xlabel('East [m]'); ylabel('North [m]');    % Remember x,y is opposide because of NED
title('Path tracking R=1000m');
legend;

