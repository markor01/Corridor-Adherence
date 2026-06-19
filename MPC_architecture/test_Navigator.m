clear; close all;

R = 2000;

% Three waypoints 

waypoints = [0,      0,     0;        % start: heading North
             5000,   2000,  pi/2;     % mid:   heading East
             10000,  8000,  0];       % goal:  heading North

corridor_widths = [300, 250];        % one per leg
speed_limits    = [10,  8  ];

nav = Navigator(waypoints, corridor_widths, speed_limits, R);
fprintf('Total path length: %.0f m\n', nav.pathLength())

% Sample path
L     = nav.pathLength();
s_vec = linspace(0, L, 500);
xp    = zeros(size(s_vec));
yp    = zeros(size(s_vec));
ey    = zeros(size(s_vec));
Um    = zeros(size(s_vec));

for k = 1:length(s_vec)
    [xp(k), yp(k), ~, ~, ey(k), Um(k)] = nav.getPathReference(s_vec(k));
end

figure; hold on; axis equal; grid on;
plot(yp, xp, 'b', 'LineWidth', 2)
xlabel('East [m]'); ylabel('North [m]')
title('Navigator — full path'); grid on;

figure;
subplot(211); plot(s_vec, ey,  'r', 'LineWidth', 1.5); ylabel('e\_y\_max [m]'); grid on;
subplot(212); plot(s_vec, Um, 'b', 'LineWidth', 1.5); ylabel('U\_max [m/s]'); grid on;
xlabel('Path variable [m]')

% Test replanning
fprintf('\n--- Replan test ---\n')
nav.replan(3000, 1000, pi/4, ...
           [10000, 8000, 0], ...
           300, 10);
fprintf('Path length after replan: %.0f m\n', nav.pathLength())