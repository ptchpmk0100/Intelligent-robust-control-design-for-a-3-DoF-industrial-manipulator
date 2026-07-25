close all;

% figure(1);
% subplot(211);
% plot(t,qds(:,1),'r',t,qs(:,1),'k:','linewidth',2);
% xlabel('time(s)');ylabel('Angle tracking for link 1');
% legend('ideal angle for link 1','angle tracking for link 1');
% subplot(212);
% plot(t,qds(:,4),'r',t,qs(:,3),'k:','linewidth',2);
% xlabel('time(s)');ylabel('Angle tracking for link 2');
% legend('ideal angle for link 2','angle tracking for link 2');
% 
% figure(2);
% subplot(211);
% plot(t,qds(:,2),'r',t,qs(:,2),'k:','linewidth',2);
% xlabel('time(s)');ylabel('Speed tracking for link 1');
% legend('ideal angle speed for link 1','angle speed tracking for link 1');
% subplot(212);
% plot(t,qds(:,5),'r',t,qs(:,4),'k:','linewidth',2);
% xlabel('time(s)');ylabel('Angle speed tracking for link 2');
% legend('ideal angle speed for link 2','angle speed tracking for link 2');

figure(1);
subplot(311);
plot(t,qds(:,1),'r',t,qs(:,1),'k:','linewidth',2);
xlabel('time(s)');ylabel('Angle tracking for link 1');
legend('ideal angle for link 1','angle tracking for link 1');
subplot(312);
plot(t,qds(:,2),'r',t,qs(:,2),'k:','linewidth',2);
xlabel('time(s)');ylabel('Angle tracking for link 2');
legend('ideal angle for link 2','angle tracking for link 2');
subplot(313);
plot(t,qds(:,3),'r',t,qs(:,3),'k:','linewidth',2);
xlabel('time(s)');ylabel('Angle tracking for link 3');
legend('ideal angle for link 3','angle tracking for link 3');

figure(3);
subplot(311);
plot(t,us1(:,1),'k','linewidth',2);
xlabel('time(s)');ylabel('control input of link 1');
subplot(312);
plot(t,us2(:,1),'k','linewidth',2);
xlabel('time(s)');ylabel('control input of link 2');
subplot(313);
plot(t,us3(:,1),'k','linewidth',2);
xlabel('time(s)');ylabel('control input of link 3');

% figure(4);
% plot(t,f(:,1),'r',t,f(:,2),'k:','linewidth',2);
% xlabel('time(s)');ylabel('f and fn');
% legend('ideal fx','estimation of fx');

% Plot individual error es1
figure(4);
plot(t, es1, 'r', 'linewidth', 2); % Plot e1 in red
xlabel('time(s)');
ylabel('Position Tracking Error es1');
legend('es1');
title('Position Tracking Error for Link 1');

% Plot individual error es2
figure(5);
plot(t, es2, 'g', 'linewidth', 2); % Plot es2 in green
xlabel('time(s)');
ylabel('Position Tracking Error es2');
legend('es2');
title('Position Tracking Error for Link 2');

% Plot individual error es3
figure(6);
plot(t, es3, 'b', 'linewidth', 2); % Plot es3 in blue
xlabel('time(s)');
ylabel('Position Tracking Error es3');
legend('es3');
title('Position Tracking Error for Link 3');

% Plot the MSE of norm (rmses_norm)
figure(7);
plot(t, rmses_norm, 'k', 'linewidth', 2); % Plot rmses_norm in black
xlabel('time(s)');
ylabel('Root Mean Square Error of Norm (rmses\_norm)');
legend('rmses\_norm');
title('Root Mean Square Error of the Norm of Tracking Errors');

% Plot all errors (es1, es2, es3) and rmses_norm combined
figure(8);
plot(t, es1, 'r', 'linewidth', 2); % Plot es1 in red
hold on;
plot(t, es2, 'g', 'linewidth', 2); % Plot es2 in green
plot(t, es3, 'b', 'linewidth', 2); % Plot es3 in blue
plot(t, rmses_norm, 'k', 'linewidth', 2); % Plot mse_norm in black
xlabel('time(s)');
ylabel('Errors and rmse\_norm');
legend('es1', 'es2', 'es3', 'rmses\_norm');
title('Position Tracking Errors (es1, es2, es3) and Root Mean Square Error Norm (rmses\_norm)');
hold off;