clear; close all;

% Test poses
x1=0; y1=0; psi1=0;           % heading North
x2=7000; y2=2000; psi2=pi/2;  % heading East
R = 2000;

[segs, L, word] = dubinsPath(x1,y1,psi1, x2,y2,psi2, R);
fprintf('Chosen word: %s,  L = %.1f m\n', word, L);

if isempty(segs)
    fprintf('No path needed, already at goal.\n')
end

% Plot via pathParam
[~,~,~,~,pathLength,~] = pathParam(0, segs);
s_vec = linspace(0, pathLength, 500);
xp = zeros(size(s_vec));
yp = zeros(size(s_vec));
for k = 1:length(s_vec)
    [xp(k), yp(k)] = pathParam(s_vec(k), segs);
end

figure; hold on; axis equal; grid on;
plot(yp, xp, 'b', 'LineWidth', 2, 'DisplayName', word);
quiver(y1, x1, R/4*sin(psi1), R/4*cos(psi1), 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 2)
quiver(y2, x2, R/4*sin(psi2), R/4*cos(psi2), 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 2)
plot(y1, x1, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8)
plot(y2, x2, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8)
xlabel('East [m]'); ylabel('North [m]');
title(sprintf('Dubins path — %s, L=%.0f m', word, L));
legend; grid on;

% Test degenerate cases
fprintf('\n--- Degenerate case tests ---\n')

% Already at goal
[~, L_degen] = dubinsPath(0,0,0, 0,0,0, R);
fprintf('Already at goal: L = %.1f (expect 0)\n', L_degen)

% Pure straight line (heading North, goal due North)
[segs_str, L_str, w_str] = dubinsPath(0,0,0, 5000,0,0, R);
fprintf('Straight ahead:  L = %.1f (expect 5000), word = %s\n', L_str, w_str)