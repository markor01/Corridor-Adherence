
clc; clear; close all;

alpha_values = [0.0001, 0.001, 0.01, 0.1, 1];
simdata_sweep = cell(length(alpha_values), 1);

for k = 1:length(alpha_values)
    alpha_cbf = alpha_values(k);
    
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
    
    e_int = 0;              % Integral term in PID heading controller
    beta_c = 0;             % Crab angle
    psi_ref = eta_0(3);     % Heading reference
    y_int = 0;              % Integral term ILOS
    path_var = 0;           % Initialize path variable
    e_int_u = 0;            % Integral term speed controller
    psi_d_prev = psi_ref;
    delta_c_prev = 0;
    alpha_cbf = 0.01;       % tuning parameter for CBF function
    
    xd = [psi_ref; 0; 0]; % 3. ordens ref modell
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dubins reference / Navigator setup
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Path A - 90 degree right turn
    
    R_min = 2000;   % minimum turning radius [m]
    
    waypoints = [0, 0, 0;               % [x, y, psi]
                 5000, 2000, pi/2;
                 5000, 10000, pi/2];
    
    corridor_widths = [120, 120];   % half width per leg
    speed_limits = [10, 10];        % per leg
    
    nav = Navigator(waypoints, corridor_widths, speed_limits, R_min);
    
    %-------------------------------------------------------------------
    % Path B - S curve
    
    % R_min = 2000;   % minimum turning radius [m]
    % d = 1000;
    % 
    % waypoints = [0,     0,    0;        % start: heading North
    %              5000,  2000, pi/2;     % end of turn 1 (right): heading East
    %              7000,  3000+d,    0;        % end of turn 2 (left): heading North again
    %              15000, 3000+d,    0];       % straight recovery leg, heading North
    % 
    % corridor_widths = [120, 120, 120];   % half width per leg
    % speed_limits = [10, 10, 10];        % per leg
    % 
    % nav = Navigator(waypoints, corridor_widths, speed_limits, R_min);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % MAIN LOOP
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    t = 0:h:T_final;                % Time vector
    nTimeSteps = length(t);         % Number of time steps
    
    simdata = zeros(nTimeSteps, 25); % Pre-allocate matrix for efficiency
    
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
    
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Guidance and control
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        [x_p, y_p, dx_p, dy_p, e_y_max, U_max, kappa] = nav.getPathReference(path_var);
    
        % Propagation of path_var
        path_var_dot = U / sqrt(dx_p^2 + dy_p^2);
        path_var = path_var + path_var_dot * h;
    
        % Calculate cross track error and path tangential angle
        [e_y,pi_p] = crossTrackErrorCurved(x_p, y_p, dx_p, dy_p, x(4), x(5));
    
        % Adaptive lookahead distance
        delta_max = 7 * L_oa;
        delta_min = 5 * L_oa;
        gamma = 0.01;
    
        delta = (delta_max - delta_min) * exp(-gamma * abs(e_y)) + delta_min;
        
        % select 'LOS' or 'ILOS'
        guidance_law = 'ILOS';
    
        if strcmp(guidance_law, 'LOS')
            % LOS guidance law
            chi_d = LOS_guidance(e_y, pi_p, delta);
            % Calculating desired heading using crab angle compensation
            psi_ref = wrapToPi(chi_d - beta_c);
        elseif strcmp(guidance_law, 'ILOS')
            % Alternatively use ILOS guidance law instead of crab angle
            % compensation. More overshoot, but no steady state error
            % speed guard
            if U > 0.5
                [psi_ref,y_int_dot] = ILOS_guidance(e_y,pi_p,y_int,delta,x(7)); 
                psi_ref = wrapToPi(psi_ref);
                y_int = y_int + h * y_int_dot;
            else
                psi_ref = pi_p;
                y_int_dot = 0;
            end
        else
            error('Guidance law must be either "LOS" or "ILOS".');
        end
    
       
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Yaw reference model
    
        % ref_model_flag = 0 -> ref_model inactive,
        % ref_model_flag = 1 -> ref_model active
        ref_model_flag = 1;
    
        if ref_model_flag
            xd_dot = ref_model(xd, psi_ref);
            xd     = xd + h*xd_dot;
            psi_d  = xd(1);
            r_d    = xd(2);
            r_d_dot = xd(3);
        else
            psi_d = psi_ref;
            r_d = 0;
            r_d_dot = 0;
        end
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Guidance level CBF corridor filter
    
        % cbf_flag = 0 -> CBF inactive,
        % cbf_flag = 1 -> CBF active
        cbf_flag = 1;
    
        if cbf_flag
            [psi_d_safe, cbf_val] = CBF_corridor(psi_d, e_y, e_y_max, pi_p, U, alpha_cbf);
        else
            psi_d_safe = psi_d;
            cbf_val = 0;
        end
    
    
        % Heading controller
       
        e_psi = ssa(x(6) - psi_d_safe);   % desired - actual
        e_r   = ssa(x(3) - r_d);
    
        % select 'FBL' or 'PID'
        heading_controller = 'FBL';
    
        if strcmp(heading_controller, 'FBL')
            delta_c = FBL_heading_control(x, psi_d_safe, r_d, r_d_dot, i);
        elseif strcmp(heading_controller, 'PID')
            [delta_c, e_int] = PID_heading(e_psi, e_r, e_int, h);
        else
            error('Heading controller must be either "FBL" or "PID".');
        end
    
    
        % Uncomment to override delta_c for either step response or max rudder
        % test
    
    %     % Step response test
    %     if i >= 5000
    %         delta_c = deg2rad(10);
    %     else
    %         delta_c = 0;
    %     end
    
    %     % Max rudder test
    %     delta_c = 40*pi/180;
    
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Closed loop speed controller
    
        u_d = U_ref;
        n_c = speed_controller(x(1), u_d, e_int_u);
        e_int_u = e_int_u + h * (U_ref - x(1)); % Integrator update
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % ship dynamics
        u = [delta_c n_c]';   
        
        % store simulation data in a table (for testing)
        simdata(i,:) = [x(1:3)' x(4:6)' x(7) x(8) u(1) u(2) u_d psi_d r_d beta beta_c x_p y_p delta psi_ref e_y_max dx_p dy_p e_y psi_d_safe cbf_val];     
     
        % Runge Kutta 4 integration
        x = rk4(@ship,h,x,u,nu_c,tau_wind);
    
        if path_var >= nav.pathLength()
            fprintf('  Reached end of path. Time simulated: %.1fs', i*h);
            break;
        end
        
        path_len = nav.pathLength();
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

    simdata_sweep{k} = simdata;

end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

colors = lines(length(alpha_values));

% Cross-track error
figure; hold on;
for k = 1:length(alpha_values)
    sd = simdata_sweep{k};
    t  = (0:size(sd,1)-1) * h;
    plot(t, sd(:,23), 'Color', colors(k,:), 'LineWidth', 1.5);
end
yline( sd(1,20), 'k--', 'LineWidth', 1.2);
yline(-sd(1,20), 'k--', 'LineWidth', 1.2);
legend(arrayfun(@(a) sprintf('\\alpha = %.3f', a), alpha_values, 'UniformOutput', false));
xlabel('Time (s)'); ylabel('Cross track error e_y (m)');
title('CBF corridor constraint, e_y vs time'); grid on;

% Commanded rudder angle
figure; hold on;
for k = 1:length(alpha_values)
    sd = simdata_sweep{k};
    t  = (0:size(sd,1)-1) * h;
    plot(t, rad2deg(sd(:,9)), 'Color', colors(k,:), 'LineWidth', 1.5);
end
yline( 40, 'k--', 'LineWidth', 1.2);
yline(-40, 'k--', 'LineWidth', 1.2);
legend(arrayfun(@(a) sprintf('\\alpha = %.3f', a), alpha_values, 'UniformOutput', false));
xlabel('Time (s)'); ylabel('Commanded rudder \delta_c (deg)');
title('CBF corridor constraint, \delta_c vs time'); grid on;

% RMS and Max |e_y| bar charts
rms_ey = zeros(1, length(alpha_values));
max_ey = zeros(1, length(alpha_values));
for k = 1:length(alpha_values)
    sd = simdata_sweep{k};
    rms_ey(k) = rms(sd(:,23));
    max_ey(k) = max(abs(sd(:,23)));
end

alpha_labels = arrayfun(@(a) sprintf('%.3f', a), alpha_values, 'UniformOutput', false);

figure;
subplot(1,2,1);
bar(rms_ey, 'FaceColor', [0.2 0.5 0.8]);
set(gca, 'XTickLabel', alpha_labels);
xlabel('\alpha'); ylabel('RMS e_y (m)');
title('RMS cross track error'); grid on;

subplot(1,2,2);
bar(max_ey, 'FaceColor', [0.8 0.3 0.2]);
set(gca, 'XTickLabel', alpha_labels);
xlabel('\alpha'); ylabel('Max |e_y| (m)');
title('Max cross track error'); grid on;