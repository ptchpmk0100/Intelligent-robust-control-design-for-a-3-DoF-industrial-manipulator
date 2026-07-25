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