clear; close all;

% Test poses
x1=0; y1=0; psi1=0;          % heading North
x2=7000; y2=2000; psi2=pi/2; % heading East
R = 2000;

words = {'RSR','LSL','RSL','LSR'};
colors = {'b','r','g','m'};

figure; hold on; axis equal; grid on;
xlabel('East (y) [m]'); ylabel('North (x) [m]');
title('Dubins words — all four CSC types');

for w = 1:4
    [segs, L] = dubinsSegment(x1,y1,psi1, x2,y2,psi2, R, words{w});

    if isempty(segs)
        fprintf('%s: infeasible\n', words{w});
        continue
    end
    fprintf('%s: L = %.1f m\n', words{w}, L);

    % Plot via pathParam to verify the full pipeline
    path_var = 0;
    path     = segs;
    [~,~,~,~,pathLength,~] = pathParam(0, path);
    s_vec = linspace(0, pathLength, 500);

    xp = zeros(size(s_vec));
    yp = zeros(size(s_vec));
    for k = 1:length(s_vec)
        [xp(k), yp(k)] = pathParam(s_vec(k), path);
    end

    % Plot in NED (North up, East right -> swap axes)
    plot(yp, xp, colors{w}, 'LineWidth', 1.5, 'DisplayName', words{w});
end

% Start and goal markers with heading arrows
quiver(y1, x1, R/4*sin(psi1), R/4*cos(psi1), 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 2)
quiver(y2, x2, R/4*sin(psi2), R/4*cos(psi2), 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 2)
plot(y1, x1, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8)
plot(y2, x2, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8)
text(y1-100, x1-200, 'start', 'FontSize', 10)
text(y2+100, x2-200, 'goal',  'FontSize', 10)

legend('Location','best');