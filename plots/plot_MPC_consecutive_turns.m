clc; clear; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load and extract data for all three disturbance levels
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
files  = {'simdata_MPC_consecutive_w_d00.mat', ...
          'simdata_MPC_consecutive_w_d08.mat', ...
          'simdata_MPC_consecutive_w_d15.mat'};

labels = {'V_c=0.0m/s', 'V_c=0.8m/s', 'V_c=1.5m/s'};
colors = {'g', 'b', 'm'};

data = cell(1,3);
for k = 1:3
    raw = load(files{k});   % loads simdata and t
    sd  = raw.simdata;

    d.t                = raw.t;

    d.u                = sd(:,1);                  % m/s
    d.v                = sd(:,2);                  % m/s
    d.r                = sd(:,3);                  % rad/s
    d.r_deg            = (180/pi) * d.r;           % deg/s
    d.x                = sd(:,4);                  % m
    d.y                = sd(:,5);                  % m
    d.psi              = sd(:,6);                  % rad
    d.psi_deg          = (180/pi) * d.psi;         % deg
    d.delta_deg        = (180/pi) * sd(:,7);       % deg
    d.n                = sd(:,8);                  % rpm
    d.n_c              = sd(:,10);                 % rpm
    d.delta_c_deg      = (180/pi) * sd(:,9);       % deg
    d.u_d              = sd(:,11);                 % m/s
    d.pi_p             = sd(:,12);                 % rad
    d.pi_p_deg         = (180/pi) * d.pi_p;        % deg
    d.x_p              = sd(:,13);                 % m
    d.y_p              = sd(:,14);                 % m
    d.y_e_max          = sd(:,15);                 % m
    d.dx_p             = sd(:,16);
    d.dy_p             = sd(:,17);
    d.y_e              = sd(:,18);                 % m
    d.infeasible_count = sd(:,19);

    data{k} = d;
end

d00 = data{1};
d08 = data{2};
d15 = data{3};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Corridor boundaries (path geometry is identical across runs, use d00)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_right = d00.x_p - d00.y_e_max .* d00.dy_p;
y_right = d00.y_p + d00.y_e_max .* d00.dx_p;
x_left  = d00.x_p + d00.y_e_max .* d00.dy_p;
y_left  = d00.y_p - d00.y_e_max .* d00.dx_p;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figure 1: Path tracking with corridor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1); hold on; axis equal; grid on;
fill([y_left; flipud(y_right)], [x_left; flipud(x_right)], ...
    [0.8 0.9 1.0], 'EdgeColor','none', 'DisplayName','Corridor');
plot(d00.y_p, d00.x_p, 'r-', 'LineWidth',1.5, 'DisplayName','Path')
plot(d00.y, d00.x, [colors{1} '--'], 'LineWidth',1.5, 'DisplayName', ['with ' labels{1}])
plot(d08.y, d08.x, [colors{2} '--'], 'LineWidth',1.5, 'DisplayName', ['with ' labels{2}])
plot(d15.y, d15.x, [colors{3} '--'], 'LineWidth',1.5, 'DisplayName', ['with ' labels{3}])
xlabel('East [m]'); ylabel('North [m]');    % Remember x,y is opposite because of NED
title('Path tracking with corridor');
legend;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figure 2: Cross-track error
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2); hold on; grid on;
fill([d00.t'; flipud(d00.t')], [d00.y_e_max; flipud(-d00.y_e_max)], [0.8 0.9 1.0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.3, 'DisplayName', 'Corridor');
plot(d00.t, d00.y_e, colors{1}, 'LineWidth', 1.5, 'DisplayName', ['Cross-track error y_e with ' labels{1}]);
plot(d08.t, d08.y_e, colors{2}, 'LineWidth', 1.5, 'DisplayName', ['Cross-track error y_e with ' labels{2}]);
plot(d15.t, d15.y_e, colors{3}, 'LineWidth', 1.5, 'DisplayName', ['Cross-track error y_e with ' labels{3}]);
plot(d00.t, d00.y_e_max, 'r--', 'LineWidth', 1.5, 'DisplayName', 'y_{e,max}');
plot(d00.t, -d00.y_e_max, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Cross-track error (m)');
title('Cross track error y_e');
legend; ylim([-1.2*max(d00.y_e_max), 1.2*max(d00.y_e_max)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figure 3: Commanded rudder angle and infeasible count
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3);
subplot(211)
plot(d00.t, d00.delta_c_deg, colors{1}, ...
     d08.t, d08.delta_c_deg, colors{2}, ...
     d15.t, d15.delta_c_deg, colors{3}, 'LineWidth',1.5);
title('Commanded rudder angle for varying disturbance'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend(['\delta_c with ' labels{1}], ['\delta_c with ' labels{2}], ['\delta_c with ' labels{3}])
grid on;

subplot(212)
styles = {'g-', 'b--', 'm:'};
plot(d00.t, d00.infeasible_count, styles{1}, ...
     d08.t, d08.infeasible_count, styles{2}, ...
     d15.t, d15.infeasible_count, styles{3}, 'LineWidth',1.5);
title('Infeasible count for varying disturbance'); xlabel('Time (s)'); ylabel('Consecutive infeasible QPs');
legend(labels{1}, labels{2}, labels{3})
grid on;