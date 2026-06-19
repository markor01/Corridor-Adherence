
clc; clear; close all;

T_final = 10000;	    % Final simulation time (s)
h = 0.1;                % Sampling time (s)
t_sim_start = tic;
last_report_percent = -1;

U_ref   = 10;           % desired surge speed (m/s)

% initial states
eta_0 = [0 0 0]';       % Velocities (linear/angular)
nu_0  = [U_ref 0 0]';       % Pose
delta_0 = 0;            % Rudder angle
n_0 = 0;                % Shaft velocity (rpm)
Qm_0 = 0;               % Engine torque
x = [nu_0' eta_0' delta_0 n_0 Qm_0]'; % x = [ u v r x y psi delta n Qm ]'

beta_c = 0;             % Crab angle
path_var = 0;           % Initialize path variable
N_mpc = 50;             % MPC prediction horizon steps
h_mpc = 1.0;            % MPC internal sample time
e_int_u = 0;            % speed controller integral term

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dubins reference / Navigator setup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path A - 90 degree right turn

% R_min = 2000;   % minimum turning radius [m]
% 
% waypoints = [0, 0, 0;               % [x, y, psi]
%              5000, 2000, pi/2;
%              5000, 10000, pi/2];
% 
% corridor_widths = [120, 120];   % half width per leg
% speed_limits = [10, 10];        % per leg
% 
% nav = Navigator(waypoints, corridor_widths, speed_limits, R_min);

%-------------------------------------------------------------------
% Path B - S curve

R_min = 2000;   % minimum turning radius [m]
d = 1000;

waypoints = [0,     0,    0;        % start: heading North
             5000,  2000, pi/2;     % end of turn 1 (right): heading East
             7000,  3000+d,    0;        % end of turn 2 (left): heading North again
             20000, 3000+d,    0];       % straight recovery leg, heading North

corridor_widths = [120, 120, 120];   % half width per leg
speed_limits = [10, 10, 10];        % per leg

nav = Navigator(waypoints, corridor_widths, speed_limits, R_min);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = 0:h:T_final;                % Time vector
nTimeSteps = length(t);         % Number of time steps

simdata = zeros(nTimeSteps, 19); % Pre-allocate matrix for efficiency

for i = 1:nTimeSteps
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Current
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Vc = 0.0;                             % Current speed
    beta_Vc = 45 * pi/180;              % Direction of current
    uc = Vc * cos(beta_Vc - x(6));
    vc = Vc * sin(beta_Vc - x(6));
    nu_c = [ uc vc 0 ]';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Wind
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Vw = 0;                             % Wind speed
    beta_Vw = 135 * pi/180; % Wind direction towards southeast

    pa = 1.247; % air density at sea level (kg/m^3)
    cy = 0.95; % dimensionless coefficient for lateral wind force
    cn = 0.15; % dimensionless coefficient for yaw wind moment
    L_oa = 161; % length overall (m)
    A_Lw = 10*L_oa; % projected lateral wind area (m^2)

    uw = Vw * cos(beta_Vw - x(6));
    vw = Vw * sin(beta_Vw - x(6));

    u_rw = x(1) - uw;
    v_rw = x(2) - vw;

    gamma_rw = -atan2(v_rw, u_rw);
    V_rw = sqrt(u_rw^2 + v_rw^2);

    CY = cy * sin(gamma_rw);
    CN = cn * sin(2*gamma_rw);

    Ywind = 0.5 * pa * V_rw^2 * A_Lw * CY;
    Nwind = 0.5 * pa * V_rw^2 * A_Lw * CN * L_oa;
    tau_wind = [0 Ywind Nwind]';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Sideslip and crab angle
    u_s = x(1);
    v_s = x(2);
    ur = u_s - uc;
    vr = v_s - vc;
    Ur = sqrt(ur^2 + vr^2);
    U = sqrt(x(1)^2 + x(2)^2);
    % speed guard sideslip angle
    if Ur > 0.5
        beta = asin(vr / Ur);
    else
        beta = 0;
    end
    % speed guard crab angle
    if U > 0.5
        beta_c = atan2(v_s, u_s);
    else
        beta_c = 0;
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % MPC
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [x_p, y_p, dx_p, dy_p, e_y_max, U_max] = nav.getPathReference(path_var);

    % Calculate cross track error and path tangential angle
    [e_y,pi_p] = crossTrackErrorCurved(x_p, y_p, dx_p, dy_p, x(4), x(5));

    % Extracting future path tangential angles pi_p
    pi_p_horizon = zeros(1, N_mpc+1);
    path_len = nav.pathLength();
    for k = 0:N_mpc
        path_var_k = min(path_var + k*U*h_mpc, path_len);
        [~, ~, dx_pk, dy_pk, ~, ~] = nav.getPathReference(path_var_k);
        pi_p_horizon(k+1) = atan2(dy_pk, dx_pk);
    end
    
    z0 = [e_y; x(6); x(3)];     % [cross track error; heading; yaw rate]
    [delta_c, infeasible_count] = MPC_heading(z0, pi_p_horizon, U, e_y_max, x(7), h_mpc, N_mpc);

    % Propagation of path_var
    path_var_dot = U / sqrt(dx_p^2 + dy_p^2);   % can drop denominator
    path_var = path_var + path_var_dot * h;
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Closed loop speed controller
    
    u_d = U_ref;
    n_c = speed_controller(x(1), u_d, e_int_u);
    e_int_u = e_int_u + h * (U_ref - x(1)); % Integrator update
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % ship dynamics
    u = [delta_c n_c]';
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [x(1:3)' x(4:6)' x(7) x(8) u(1) u(2) u_d pi_p x_p y_p e_y_max dx_p dy_p e_y infeasible_count];
 
    % Runge Kutta 4 integration
    x = rk4(@ship,h,x,u,nu_c,tau_wind);

    % Stop if reached end of path
    if path_var >= nav.pathLength()
        fprintf('  Reached end of path. Time simulated: %.1fs', i*h);
        break;
    end

    percent_path = 100 * path_var / path_len;
    percent_bucket = floor(percent_path / 5) * 5;   % report every 5% of path
    
    if percent_bucket > last_report_percent
        last_report_percent = percent_bucket;
        elapsed = toc(t_sim_start);
        if percent_path > 0.5
            eta = elapsed / (percent_path/100) * (1 - percent_path/100);
        else
            eta = NaN;
        end
        fprintf('  Path: %3.0f%%  |  Sim time: %6.1fs  |  Elapsed: %5.1fs  |  ETA: %5.1fs\n', ...
            percent_path, i*h, elapsed, eta);
    end


end

simdata = simdata(1:i,:);
t = t(1:i);

save('simdata_MPC_consecutive_w_d20.mat', 'simdata', 't');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
plot(t, e_y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Cross-track error e_y');
plot(t, e_y_max, 'r--', 'LineWidth', 1.5, 'DisplayName', 'e_{y,max}');
plot(t, -e_y_max, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Cross-track error (m)');
title('CBF corridor constraint - e_y vs boundary');
legend; ylim([-1.2*max(e_y_max), 1.2*max(e_y_max)]);

figure(3); hold on; grid on;
plot(t, infeasible_count, 'b', 'LineWidth',1.5);
title('Infeasible count'); xlabel('Time (s)'); ylabel('Consecutive infeasible QPs');

figure(4)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocity'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('actual surge','desired surge')
subplot(312)
plot(t,n,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed'); xlabel('Time (s)'); ylabel('Motor speed (RPM)');
legend('actual RPM','commanded RPM')
subplot(313)
plot(t,delta_deg,t,delta_c_deg,'linewidth',2);
title('Actual and commanded rudder angle'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('actual rudder angle','commanded rudder angle')

