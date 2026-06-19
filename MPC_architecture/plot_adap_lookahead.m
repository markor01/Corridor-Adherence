clf;

L_oa = 161;
delta_max = 5 * L_oa;
delta_min = 2 * L_oa;
gamma1 = 0.01;
gamma2 = 0.03;
gamma3 = 0.05;
gamma4 = 0.1;
gamma5 = 0.5;

delta1 = zeros(1,100);
delta2 = zeros(1,100);
delta3 = zeros(1,100);
delta4 = zeros(1,100);
delta5 = zeros(1,100);

y_e = 0:99;

for i = y_e

    delta1(i+1) = (delta_max - delta_min) * exp(-gamma1 * abs(i)) + delta_min;
    delta2(i+1) = (delta_max - delta_min) * exp(-gamma2 * abs(i)) + delta_min;
    delta3(i+1) = (delta_max - delta_min) * exp(-gamma3 * abs(i)) + delta_min;
    delta4(i+1) = (delta_max - delta_min) * exp(-gamma4 * abs(i)) + delta_min;
    delta5(i+1) = (delta_max - delta_min) * exp(-gamma5 * abs(i)) + delta_min;
end

figure(1);
hold on;
grid on;
h1 = plot(y_e, delta1, 'LineWidth', 1, 'Color', 'b');
h2 = plot(y_e, delta2, 'LineWidth', 1, 'Color', 'g');
h3 = plot(y_e, delta3, 'LineWidth', 1, 'Color', 'y');
h4 = plot(y_e, delta4, 'LineWidth', 1, 'Color', 'c');
h5 = plot(y_e, delta5, 'LineWidth', 1, 'Color', 'm');
title('Lookahead distance \Delta for different values of \gamma')
lgd = legend([h1 h2 h3 h4 h5], {'\gamma=0.01','\gamma=0.03','\gamma=0.05','\gamma=0.1', '\gamma=0.5'});
lgd.AutoUpdate = 'off';
xlabel('y_e^p'); ylabel('\Delta')
yline(delta_min, '--r', 'Min \Delta', 'LineWidth', 1.5);
yline(delta_max, '--r', 'Max \Delta', 'LineWidth', 1.5);
ylim([delta_min-50, delta_max+50]);
