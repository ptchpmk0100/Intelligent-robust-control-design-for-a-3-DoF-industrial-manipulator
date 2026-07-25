close all

% minLength = min([length(t), length(e1), length(e2), length(e3), length(rmse_norm), ...
%                  length(es1), length(es2), length(es3), length(rmses_norm), ...
%                  length(ea1), length(ea2), length(ea3), length(rmsea_norm), ...
%                  length(qd), length(q), length(qa), length(qs)]);

% t = t(1:minLength);
% e1 = e1(1:minLength);
% e2 = e2(1:minLength);
% e3 = e3(1:minLength);
% rmse_norm = rmse_norm(1:minLength);
% 
% es1 = es1(1:minLength);
% es2 = es2(1:minLength);
% es3 = es3(1:minLength);
% rmses_norm = rmses_norm(1:minLength);
% 
% ea1 = ea1(1:minLength);
% ea2 = ea2(1:minLength);
% ea3 = ea3(1:minLength);
% rmsea_norm = rmsea_norm(1:minLength);

% minLength = min([length(q), length(qa), length(qs)]);
% minLength = min([length(q), length(qa)]);

% t = t(1:minLength);

% qd = qd(1:length(t),:);
% qda = qda(1:length(t),:);
% qds = qds(1:length(t),:);
% q = q(1:length(t),:);
% qa = qa(1:length(t),:);
% qs = qs(1:length(t),:);

%---------------Sine Wave-----------%
% qd1=0.1*sin(t);
% qd2=0.1*sin(t);
% qd3=0.1*sin(t);
qd1=0.5*sin(t);
qd2=0.5*sin(t);
qd3=0.5*sin(t);
qd = [qd1, qd2, qd3];

%---------------Const Wave-----------%
% qd1 = 1*ones(length(t),1);
% qd2 = 1.5*ones(length(t),1);
% qd3 = 2*ones(length(t),1);
% qd = [qd1, qd2, qd3];

%-------------Square Wave-----------%
% qd1 = 0.1 * square(t);
% qd2 = 0.1 * square(t);
% qd3 = 0.1 * square(t);
% qd1 = 0.5*square(t);
% qd2 = 0.5*square(t);
% qd3 = 0.5*square(t);
% qd = [qd1, qd2, qd3];

% 
% figure(1)
% plot(t,qd(:,1),'k:','linewidth',4);
% hold on
% plot(t,q(:,1),'r',t,qa(:,1),'g',t,qs(:,1),'b','linewidth',3)
% ylim([-1 1]);
% xlim([0 10]);
% xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1.5);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 18);  
% legend('Ideal Angle for Link 1', 'Proposed Controller','NTSMC-RBFNN', 'SMC-RBFNN', 'FontSize', 10);
% % title('Angle Tracking for Link 1', 'FontSize', 12, 'FontWeight', 'bold');
% 
% figure(2)
% plot(t,qd(:,2),'k:','linewidth',4);
% hold on
% plot(t,q(:,3),'r',t,qa(:,3),'g',t,qs(:,3),'b','linewidth',3)
% ylim([-1 1]);
% xlim([0 10]);
% xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1.5);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 18);  
% legend('Ideal Angle for Link 2', 'Proposed Controller','NTSMC-RBFNN', 'SMC-RBFNN', 'FontSize', 10);
% % title('Angle Tracking for Link 2', 'FontSize', 12, 'FontWeight', 'bold');
% 
% figure(3)
% plot(t,qd(:,3),'k:','linewidth',4);
% hold on
% plot(t,q(:,5),'r',t,qa(:,5),'g',t,qs(:,5),'b','linewidth',3)
% ylim([-1 1]);
% xlim([0 10]);
% xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1.5);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 18);  
% legend('Ideal Angle for Link 3', 'Proposed Controller','NTSMC-RBFNN', 'SMC-RBFNN', 'FontSize', 10);
% % title('Angle Tracking for Link 3', 'FontSize', 12, 'FontWeight', 'bold');
% 
% figure(4)
% plot(t, rmse_norm, 'r', 'linewidth', 3);
% hold on
% plot(t, rmsea_norm, 'g', 'linewidth', 3);
% plot(t, rmses_norm, 'b', 'linewidth', 3);
% ylim([0 1.6]);
% xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('RMSE [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1.5);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 18);  
% legend('Proposed Controller', 'NTSMC-RBFNN', 'SMC-RBFNN', 'FontSize', 10);
% % title('Tracking Errors', 'FontSize', 14, 'FontWeight', 'bold');

figure(1)
plot(t,qd(:,1),'k:','linewidth',2);
hold on
plot(t,q(:,1),'r',t,qa(:,1),'g','linewidth',3)
ylim([-1 1]);
% xlim([0 10]);
xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% Make x and y axes bold and increase font size
set(gca, 'LineWidth', 1.5);
set(gca, 'FontWeight', 'bold');
set(gca, 'FontSize', 18);  
legend('Ideal Angle for Link 1', 'Proposed Controller','Fuzzy-PID-SMC','FontSize', 10);
% title('Angle Tracking for Link 1', 'FontSize', 12, 'FontWeight', 'bold');

figure(2)
plot(t,qd(:,2),'k:','linewidth',2);
hold on
plot(t,q(:,2),'r',t,qa(:,2),'g','linewidth',3)
ylim([-1 1]);
% xlim([0 10]);
xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% Make x and y axes bold and increase font size
set(gca, 'LineWidth', 1.5);
set(gca, 'FontWeight', 'bold');
set(gca, 'FontSize', 18);  
legend('Ideal Angle for Link 2', 'Proposed Controller','Fuzzy-PID-SMC', 'FontSize', 10);
% title('Angle Tracking for Link 2', 'FontSize', 12, 'FontWeight', 'bold');

figure(3)
plot(t,qd(:,3),'k:','linewidth',2);
hold on
plot(t,q(:,3),'r',t,qa(:,3),'g','linewidth',3)
ylim([-1 1]);
% xlim([0 10]);
xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% Make x and y axes bold and increase font size
set(gca, 'LineWidth', 1.5);
set(gca, 'FontWeight', 'bold');
set(gca, 'FontSize', 18);  
legend('Ideal Angle for Link 3', 'Proposed Controller', 'Fuzzy-PID-SMC', 'FontSize', 10);
% title('Angle Tracking for Link 3', 'FontSize', 12, 'FontWeight', 'bold');

figure(4)
plot(t,e1,'r','linewidth',2);
hold on
plot(t,ea1,'g','linewidth',3)
% ylim([-0.015 0.04]);
% xlim([0 10]);
xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Tracking error [rad]', 'FontSize', 10, 'FontWeight', 'bold');
% Make x and y axes bold and increase font size
set(gca, 'LineWidth', 1);
set(gca, 'FontWeight', 'bold');
set(gca, 'FontSize', 16);  
legend('Proposed controller','Fuzzy-PID-SMC', 'FontSize', 10);
% title('Angle Tracking for Link 1', 'FontSize', 12, 'FontWeight', 'bold');

figure(5)
plot(t,e2,'r','linewidth',2);
hold on
plot(t,ea2,'g','linewidth',3)
% ylim([-0.015 0.04]);
% xlim([0 10]);
xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Tracking error [rad]', 'FontSize', 10, 'FontWeight', 'bold');
% Make x and y axes bold and increase font size
set(gca, 'LineWidth', 1);
set(gca, 'FontWeight', 'bold');
set(gca, 'FontSize', 16);  
legend('Proposed controller','Fuzzy-PID-SMC', 'FontSize', 10);
% title('Angle Tracking for Link 1', 'FontSize', 12, 'FontWeight', 'bold');

figure(6)
plot(t,e3,'r','linewidth',2);
hold on
plot(t,ea3,'g','linewidth',3)
% ylim([-0.015 0.04]);
% xlim([0 10]);
xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Tracking error [rad]', 'FontSize', 10, 'FontWeight', 'bold');
% Make x and y axes bold and increase font size
set(gca, 'LineWidth', 1);
set(gca, 'FontWeight', 'bold');
set(gca, 'FontSize', 16);  
legend('Proposed controller','Fuzzy-PID-SMC', 'FontSize', 10);
% title('Angle Tracking for Link 1', 'FontSize', 12, 'FontWeight', 'bold');

