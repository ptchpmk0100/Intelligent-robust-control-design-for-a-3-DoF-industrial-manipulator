close all;

% Figure 1: Position Tracking for 3 Links
figure(1);

subplot(311);
plot(t, qda(:, 1), 'k', t, qa(:, 1), 'r:', 'linewidth', 2);
xlabel('time(s)');
ylabel('Position tracking of link1');
legend('Ideal position signal', 'Tracking signal');

subplot(312);
plot(t, qda(:, 2), 'k', t, qa(:, 2), 'r:', 'linewidth', 2);
xlabel('time(s)');
ylabel('Position tracking of link2');
legend('Ideal position signal', 'Tracking signal');

subplot(313);
plot(t, qda(:, 1), 'k', t, qa(:, 3), 'r:', 'linewidth', 2);
xlabel('time(s)');
ylabel('Position tracking of link3');
legend('Ideal position signal', 'Tracking signal');

% % Figure 2: Practical F and Estimated F for 3 Links
% figure(2);
% 
% subplot(311);
% plot(t, y(:, 7), 'k', t, u(:, 4), 'r:', 'linewidth', 2);
% xlabel('time(s)');
% ylabel('F and Fc of link1');
% legend('Practical F', 'Estimation of F');
% 
% subplot(312);
% plot(t, y(:, 8), 'k', t, u(:, 5), 'r:', 'linewidth', 2);
% xlabel('time(s)');
% ylabel('F and Fc of link2');
% legend('Practical F', 'Estimation of F');
% 
% subplot(313);
% plot(t, y(:, 9), 'k', t, u(:, 6), 'r:', 'linewidth', 2);
% xlabel('time(s)');
% ylabel('F and Fc of link3');
% legend('Practical F', 'Estimation of F');

% Figure 3: Control Inputs for 3 Links
figure(3);

subplot(311);
plot(t, ua1, 'r', 'linewidth', 2);
xlabel('time(s)');
ylabel('Control input of Link1');

subplot(312);
plot(t, ua2, 'r', 'linewidth', 2);
xlabel('time(s)');
ylabel('Control input of Link2');

subplot(313);
plot(t, ua3, 'r', 'linewidth', 2);
xlabel('time(s)');
ylabel('Control input of Link3');

% Figure 4: Error Tracking 3 links
figure(4);

subplot(411);
plot(t, ea1, 'r', 'linewidth', 2);
xlabel('time(s)');
ylabel('Error Tracking of Link1');

subplot(312);
plot(t, ea2, 'r', 'linewidth', 2);
xlabel('time(s)');
ylabel('Error Tracking of Link2');

subplot(313);
plot(t, ea3, 'r', 'linewidth', 2);
xlabel('time(s)');
ylabel('Error Tracking of Link3');

