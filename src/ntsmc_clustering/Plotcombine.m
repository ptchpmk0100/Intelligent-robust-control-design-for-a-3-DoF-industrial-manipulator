% Find the minimum length among the vectors
% minLength = min([length(t), length(e1), length(e2), length(e3), length(en_norm), ...
%                  length(es1), length(es2), length(es3), length(es_norm)]);
minLength = min([length(t), length(e1), length(e2), length(e3), length(rmse_norm), ...
                 length(es1), length(es2), length(es3), length(rmses_norm), ...
                 length(ea1), length(ea2), length(ea3), length(rmsea_norm)]);

% Trim all vectors to the same minimum length
t = t(1:minLength);
e1 = e1(1:minLength);
e2 = e2(1:minLength);
e3 = e3(1:minLength);
% en_norm = en_norm(1:minLength);
rmse_norm = rmse_norm(1:minLength);

es1 = es1(1:minLength);
es2 = es2(1:minLength);
es3 = es3(1:minLength);
% es_norm = es_norm(1:minLength);
rmses_norm = rmses_norm(1:minLength);

ea1 = ea1(1:minLength);
ea2 = ea2(1:minLength);
ea3 = ea3(1:minLength);
% es_norm = es_norm(1:minLength);
rmsea_norm = rmsea_norm(1:minLength);

%-----------------COMBINEd---------------%
figure(1)
plot(t, e1, 'r', 'linewidth', 2); 
hold on;
plot(t, ea1, 'g', 'linewidth', 2);
plot(t, es1, 'b', 'linewidth', 2);
xlabel('time (s)');
ylabel('Error Tracking');
legend('newNTSMC', 'prevNTSMC', 'SMC');
title('RMSE Tracking between diffenrent methods');
hold off;

figure(2)
plot(t, e2, 'r', 'linewidth', 2); 
hold on;
plot(t, ea2, 'g', 'linewidth', 2);
plot(t, es2, 'b', 'linewidth', 2);
xlabel('time (s)');
ylabel('Error Tracking');
legend('newNTSMC', 'prevNTSMC', 'SMC');
title('Error Tracking between diffenrent methods');
hold off;

figure(3)
plot(t, e3, 'r', 'linewidth', 2); 
hold on;
plot(t, ea3, 'g', 'linewidth', 2);
plot(t, es3, 'b', 'linewidth', 2);
xlabel('time (s)');
ylabel('Error Tracking');
legend('newNTSMC', 'prevNTSMC', 'SMC');
title('Error Tracking between diffenrent methods');
hold off;

figure(4)
plot(t, rmse_norm, 'r', 'linewidth', 2); 
hold on;
plot(t, rmsea_norm, 'g', 'linewidth', 2);
plot(t, rmses_norm, 'b', 'linewidth', 2);
xlabel('time (s)');
ylabel('RMSE');
legend('newNTSMC', 'prevNTSMC', 'SMC');
title('RMSE Tracking between diffenrent methods');
hold off;

%------------------------------------------------------------------------------------

%--------------------------------LINK 1--------------------------------%
% figure(1);
% subplot(4, 1, 1);
% plot(t, e1, 'r', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('Error Tracking for Link 1');
% legend('e1');
% 
% subplot(4,1,2);
% plot(t, ea1, 'g', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('Error Tracking for Link 1');
% legend('ea1');
% 
% subplot(4,1,3);
% plot(t, es1, 'b', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('Error Tracking for Link 1');
% legend('es1');
% 
% subplot(4,1,4);
% plot(t, e1, 'r', 'linewidth', 2); 
% hold on;
% plot(t, ea1, 'g', 'linewidth', 2);
% plot(t, es1, 'b', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('Error Tracking for Link 1');
% legend('e1', 'ea1', 'es1');
% title('Comparison of NTSMC, prevNTSMC, SMC');
% hold off;
% 
% %--------------------------------LINK 2--------------------------------%
% figure(2);
% subplot(4, 1, 1);
% plot(t, e2, 'r', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('Error Tracking for Link 2');
% legend('e2');
% 
% subplot(4,1,2);
% plot(t, ea2, 'g', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('Error Tracking for Link 2');
% legend('ea2');
% 
% subplot(4,1,3);
% plot(t, es2, 'b', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('Error Tracking for Link 2');
% legend('es2');
% 
% subplot(4,1,4);
% plot(t, e2, 'r', 'linewidth', 2); 
% hold on;
% plot(t, ea2, 'g', 'linewidth', 2);
% plot(t, es2, 'b', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('Error Tracking for Link 2');
% legend('e2', 'ea2', 'es2');
% title('Comparison of NTSMC, prevNTSMC, SMC');
% hold off;
% 
% %--------------------------------LINK 3--------------------------------%
% figure(3);
% subplot(4, 1, 1);
% plot(t, e3, 'r', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('Error Tracking for Link 3');
% legend('e3');
% 
% subplot(4,1,2);
% plot(t, ea3, 'g', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('Error Tracking for Link 3');
% legend('ea3');
% 
% subplot(4,1,3);
% plot(t, es3, 'b', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('Error Tracking for Link 3');
% legend('es3');
% 
% subplot(4,1,4);
% plot(t, e3, 'r', 'linewidth', 2); 
% hold on;
% plot(t, ea3, 'g', 'linewidth', 2);
% plot(t, es3, 'b', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('Error Tracking for Link 3');
% legend('e3', 'ea3', 'es3');
% title('Comparison of NTSMC, prevNTSMC, SMC');
% hold off;
% 
% %--------------------------------RMSE--------------------------------%
% figure(4);
% subplot(4, 1, 1);
% plot(t, rmse_norm, 'r', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('RMSE NTSMC');
% legend('rmse');
% 
% subplot(4,1,2);
% plot(t, rmsea_norm, 'g', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('RMSE prevNTSMC');
% legend('rmsea');
% 
% subplot(4,1,3);
% plot(t, rmses_norm, 'b', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('RMSE SMC');
% legend('rmses');
% 
% subplot(4,1,4);
% plot(t, rmse_norm, 'r', 'linewidth', 2); 
% hold on;
% plot(t, rmsea_norm, 'g', 'linewidth', 2);
% plot(t, rmses_norm, 'b', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('RMSE');
% legend('rmse', 'rmsea', 'rmses');
% title('Comparison of NTSMC, prevNTSMC, SMC');
% hold off;
% 
% %--------------------------------RMSE alone--------------------------------%
% figure(5)
% plot(t, rmse_norm, 'r', 'linewidth', 2); 
% hold on;
% plot(t, rmsea_norm, 'g', 'linewidth', 2);
% plot(t, rmses_norm, 'b', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('RMSE');
% legend('newNTSMC', 'prevNTSMC', 'SMC');
% title('Comparison of newNTSMC, prevNTSMC, SMC');
% hold off;

%--------------------------------------------------------------------------------------------

% % Create a figure with 4 subplots
% figure;
% 
% % Subplot 1: e1 vs. es1
% subplot(4, 1, 1);
% plot(t, e1, 'r', 'linewidth', 2); % Plot e1 in red
% hold on;
% plot(t, ea1, 'g', 'linewidth', 2); % Plot es1 in dashed red
% plot(t, es1, 'b', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('Error Tracking for Link 1');
% legend('e1', 'ea1', 'es1');
% title('Comparison of NTSMC, prevNTSMC, SMC');
% hold off;
% 
% % Subplot 2: e2 vs. es2
% subplot(4, 1, 2);
% plot(t, e2, 'r', 'linewidth', 2); % Plot e2 in green
% hold on;
% plot(t, ea2, 'g', 'linewidth', 2); % Plot es2 in dashed green
% plot(t, es2, 'b', 'linewidth', 2); 
% xlabel('time (s)');
% ylabel('Error Tracking for link 2');
% legend('e2', 'ea2', 'ea2');
% title('Comparison of NTSMC, prevNTSMC, SMC');
% hold off;
% 
% % Subplot 3: e3 vs. es3
% subplot(4, 1, 3);
% plot(t, e3, 'r', 'linewidth', 2); % Plot e3 in blue
% hold on;
% plot(t, ea3, 'g', 'linewidth', 2); % Plot es3 in dashed blue
% plot(t, es3, 'b', 'linewidth', 2); % Plot es3 in dashed blue
% xlabel('time (s)');
% ylabel('Error Tracking for link 3');
% legend('e3', 'ea3', 'es3');
% title('Comparison of NTSMC, prevNTSMC, SMC');
% hold off;
% 
% % Subplot 4: en_norm vs. es_norm
% % subplot(4, 1, 4);
% % plot(t, en_norm, 'k', 'linewidth', 2); % Plot en_norm in black
% % hold on;
% % plot(t, es_norm, 'k--', 'linewidth', 2); % Plot es_norm in dashed black
% % xlabel('time (s)');
% % ylabel('Norm of Error');
% % legend('en\_norm', 'es\_norm');
% % title('Comparison of en\_norm vs. es\_norm');
% % hold off;
% 
% % Subplot 4: en_norm vs. es_norm
% subplot(4, 1, 4);
% plot(t, rmse_norm, 'r', 'linewidth', 2); % Plot en_norm in black
% hold on;
% plot(t, rmsea_norm, 'g', 'linewidth', 2); % Plot es_norm in dashed black
% plot(t, rmses_norm, 'b', 'linewidth', 2);
% xlabel('time (s)');
% ylabel('Root Mean Square of Error');
% legend('rmse\_norm', 'rmsea\_norm', 'rmses_norm');
% title('Comparison of Root Mean Square');
% hold off;
% 
% % Adjust layout for better visualization
% sgtitle('Comparison of Tracking Errors: NTSMC vs. prevNTSMC vs. SMC');
