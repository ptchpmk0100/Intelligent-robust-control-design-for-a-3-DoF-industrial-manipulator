close all;

figure(1);
subplot(311);
plot(t,qd(:,1),'r',t,q(:,1),'k:','linewidth',2);
xlabel('time(s)');ylabel('Angle tracking for link 1');
legend('ideal angle for link 1','angle tracking for link 1');
subplot(312);
plot(t,qd(:,2),'r',t,q(:,2),'k:','linewidth',2);
xlabel('time(s)');ylabel('Angle tracking for link 2');
legend('ideal angle for link 2','angle tracking for link 2');
subplot(313);
plot(t,qd(:,3),'r',t,q(:,3),'k:','linewidth',2);
xlabel('time(s)');ylabel('Angle tracking for link 3');
legend('ideal angle for link 3','angle tracking for link 3');

% figure(2);
% subplot(311);
% plot(t,qd(:,2),'r',t,q(:,2),'k:','linewidth',2);
% xlabel('time(s)');ylabel('Speed tracking for link 1');
% legend('ideal angle speed for link 1','speed tracking for link 1');
% subplot(312);
% plot(t,qd(:,5),'r',t,q(:,4),'k:','linewidth',2);
% xlabel('time(s)');ylabel('Angle speed tracking for link 2');
% legend('ideal angle speed for link 2','speed tracking for link 2');
% subplot(313);
% plot(t,qd(:,8),'r',t,q(:,6),'k:','linewidth',2);
% xlabel('time(s)');ylabel('Angle speed tracking for link 3');
% legend('ideal angle speed for link 3','speed tracking for link 3');

figure(3);
subplot(311);
plot(t,u1(:,1),'r','linewidth',2);
xlabel('time(s)');ylabel('control input of link 1');
subplot(312);
plot(t,u2(:,1),'g','linewidth',2);
xlabel('time(s)');ylabel('control input of link 2');
subplot(313);
plot(t,u3(:,1),'b','linewidth',2);
xlabel('time(s)');ylabel('control input of link 3');

% figure(4);
% plot(t, e1, 'r', 'linewidth', 2); % Plot e1 in red
% hold on;
% plot(t, e2, 'g', 'linewidth', 2); % Plot e2 in green
% plot(t, e3, 'b', 'linewidth', 2); % Plot e3 in blue
% plot(t, en_norm, 'k', 'linewidth', 2); % Plot en_norm in black
% xlabel('time(s)');
% ylabel('Position Tracking Errors');
% legend('e1', 'e2', 'e3', 'en\_norm');
% title('Position Tracking Errors and Norm of Error');
% hold off;

% Plot individual error e1
figure(4);
plot(t, e1, 'r', 'linewidth', 2); % Plot e1 in red
xlabel('time(s)');
ylabel('Position Tracking Error e1');
legend('e1');
title('Position Tracking Error for Link 1');

% Plot individual error e2
figure(5);
plot(t, e2, 'g', 'linewidth', 2); % Plot e2 in green
xlabel('time(s)');
ylabel('Position Tracking Error e2');
legend('e2');
title('Position Tracking Error for Link 2');

% Plot individual error e3
figure(6);
plot(t, e3, 'b', 'linewidth', 2); % Plot e3 in blue
xlabel('time(s)');
ylabel('Position Tracking Error e3');
legend('e3');
title('Position Tracking Error for Link 3');

% Plot the MSE of norm (mse_norm)
figure(7);
plot(t, rmse_norm, 'k', 'linewidth', 2); % Plot mse_norm in black
xlabel('time(s)');
% ylabel('Root Mean Square Error of Norm (rmse\_norm)');
ylabel('RMS of Tracking Error rmse');
legend('rmse');
title('RMS of Tracking Errors');

% Plot all errors (e1, e2, e3) and mse_norm combined
figure(8);
plot(t, e1, 'r', 'linewidth', 2); % Plot e1 in red
hold on;
plot(t, e2, 'g', 'linewidth', 2); % Plot e2 in green
plot(t, e3, 'b', 'linewidth', 2); % Plot e3 in blue
plot(t, rmse_norm, 'k', 'linewidth', 2); % Plot mse_norm in black
xlabel('time(s)');
ylabel('Errors and RMSE');
legend('e1', 'e2', 'e3', 'rmse');
title('Position Tracking Errors (e1, e2, e3) and RMS Error (rmse)');
hold off;


% % Additional Figures: Sliding Surface and Norm of Sliding Surface
% figure(5);
% subplot(311);
% plot(t, s(1, :), 'r', 'linewidth', 2);  % Sliding surface for Joint 1
% xlabel('time(s)');
% ylabel('s_1');
% title('Sliding Surface for Joint 1');
% 
% subplot(312);
% plot(t, s(2, :), 'g', 'linewidth', 2);  % Sliding surface for Joint 2
% xlabel('time(s)');
% ylabel('s_2');
% title('Sliding Surface for Joint 2');
% 
% subplot(313);
% plot(t, s(3, :), 'b', 'linewidth', 2);  % Sliding surface for Joint 3
% xlabel('time(s)');
% ylabel('s_3');
% title('Sliding Surface for Joint 3');
% 
% figure(6);
% s_norm = vecnorm(s, 2, 1);  % Compute the 2-norm of the sliding surface across joints
% plot(t, s_norm, 'k', 'linewidth', 2);
% xlabel('time(s)');
% ylabel('||s||');
% title('Norm of Sliding Surface for All Joints');

% figure(1);
% subplot(311);
% plot(t,qds(:,1),'r',t,qs(:,1),'k:','linewidth',2);
% xlabel('time(s)');ylabel('SMC - Angle tracking for joint 1');
% legend('ideal angle for link 1','angle tracking for joint 1');
% subplot(312);
% plot(t,qds(:,4),'r',t,qs(:,3),'k:','linewidth',2);
% xlabel('time(s)');ylabel('SMC - Angle tracking for joint 2');
% legend('ideal angle for link 2','angle tracking for joint 2');
% subplot(313);
% plot(t,qds(:,7),'r',t,qs(:,5),'k:','linewidth',2);
% xlabel('time(s)');ylabel('SMC - Angle tracking for link 3');
% legend('ideal angle for link 3','angle tracking for link 3');
% 
% figure(2);
% subplot(311);
% plot(t,qds(:,2),'r',t,qs(:,2),'k:','linewidth',2);
% xlabel('time(s)');ylabel('SMC - Speed tracking for joint 1');
% legend('ideal angle speed for link 1','speed tracking for joint 1');
% subplot(312);
% plot(t,qds(:,5),'r',t,qs(:,4),'k:','linewidth',2);
% xlabel('time(s)');ylabel('SMC - Speed tracking for joint 2');
% legend('ideal angle speed for link 2','speed tracking for joint 2');
% subplot(313);
% plot(t,qds(:,8),'r',t,qs(:,6),'k:','linewidth',2);
% xlabel('time(s)');ylabel('SMC - Speed tracking for joint 3');
% legend('ideal angle speed for link 3','speed tracking for joint 3');
% 
% figure(3);
% subplot(311);
% plot(t,u1(:,1),'r','linewidth',2);
% xlabel('time(s)');ylabel('SMC control input of joint 1');
% subplot(312);
% plot(t,u2(:,1),'g','linewidth',2);
% xlabel('time(s)');ylabel('SMC control input of joint 2');
% subplot(313);
% plot(t,u3(:,1),'b','linewidth',2);
% xlabel('time(s)');ylabel('SMC control input of joint 3');
% 
% figure(4);
% plot(t, es1, 'r', 'linewidth', 2); % Plot e1 in red
% hold on;
% plot(t, es2, 'g', 'linewidth', 2); % Plot e2 in green
% plot(t, es3, 'b', 'linewidth', 2); % Plot e3 in blue
% plot(t, ens_norm, 'k', 'linewidth', 2); % Plot en_norm in black
% xlabel('time(s)');
% ylabel('Position Tracking Errors');
% legend('es1', 'es2', 'es3', 'ens\_norm');
% title('SMC Position Tracking Errors and Norm of Error');
% hold off;