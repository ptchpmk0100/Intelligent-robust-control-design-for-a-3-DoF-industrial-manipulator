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
% qd1=0.5*sin(t);
% qd2=0.5*sin(t);
% qd3=0.5*sin(t);
% qd = [qd1, qd2, qd3];

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

% colorSet = {
%     [1 0 0],          % red
%     [0 1 0],          % green
%     [0 0 1],          % blue
%     [1 0.5 0],        % orange
%     [0.13 0.7 0.67],  % teal
%     [1 0 1],          % magenta
%     [1 0.84 0],       % gold
%     [0 0 0.5],        % dark blue
%     % [1 0.4 0.7]       % pink
%     [0.6 1 0.6]       % light green
% };
% 
% figure(1)
% plot(t,qd(:,1),'k:','linewidth',2);
% hold on
% plot(t,q(:,1),'r',t,qa(:,1),'g',t,qs(:,1),'b','linewidth',3)
% ylim([-1 1]);
% % xlim([0 10]);
% xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1.5);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 18);  
% legend('Ideal Angle for Link 1', 'Proposed Controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);
% % title('Angle Tracking for Link 1', 'FontSize', 12, 'FontWeight', 'bold');
% 
% figure(2)
% plot(t,qd(:,2),'k:','linewidth',2);
% hold on
% plot(t,q(:,2),'c',t,qa(:,2),'g',t,qs(:,2),'b','linewidth',3)
% ylim([-1 1]);
% % xlim([0 10]);
% xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1.5);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 18);  
% legend('Ideal Angle for Link 1', 'Proposed Controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);
% % title('Angle Tracking for Link 2', 'FontSize', 12, 'FontWeight', 'bold');
% 
% figure(3)
% plot(t,qd(:,3),'k:','linewidth',2);
% hold on
% plot(t,q(:,3),'m',t,qa(:,3),'g',t,qs(:,2),'b','linewidth',3)
% ylim([-1 1]);
% % xlim([0 10]);
% xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1.5);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 18);  
% legend('Ideal Angle for Link 1', 'Proposed Controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);
% % title('Angle Tracking for Link 3', 'FontSize', 12, 'FontWeight', 'bold');
% 
% figure(4)
% plot(t,e1,'r','linewidth',2);
% hold on
% plot(t,ea1,'g','linewidth',2)
% plot(t,es1,'b','linewidth',2)
% % ylim([-0.015 0.04]);
% % xlim([0 10]);
% xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
% ylabel('Tracking Error [rad]', 'FontSize', 10, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 16);  
% legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);
% % title('Angle Tracking for Link 1', 'FontSize', 12, 'FontWeight', 'bold');
% 
% figure(5)
% plot(t,e2,'c','linewidth',2);
% hold on
% plot(t,ea2,'g','linewidth',2)
% plot(t,es2,'b','linewidth',2)
% % ylim([-0.015 0.04]);
% % xlim([0 10]);
% xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
% ylabel('Tracking Error [rad]', 'FontSize', 10, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 16);  
% legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);
% % title('Angle Tracking for Link 1', 'FontSize', 12, 'FontWeight', 'bold');
% 
% figure(6)
% plot(t,e3,'m','linewidth',2);
% hold on
% plot(t,ea3,'g','linewidth',2)
% plot(t,es3,'b','linewidth',2)
% % ylim([-0.015 0.04]);
% % xlim([0 10]);
% xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
% ylabel('Tracking Error [rad]', 'FontSize', 10, 'FontWeight', 'bold');
% % Make x and y axes bold and increase font size
% set(gca, 'LineWidth', 1);
% set(gca, 'FontWeight', 'bold');
% set(gca, 'FontSize', 16);  
% legend('Proposed controller','Fuzzy-PID-SMC','SMC','FontSize', 10);
% % title('Angle Tracking for Link 1', 'FontSize', 12, 'FontWeight', 'bold');



%-----------------------------------------------------------------------------%

%---------------Sine Wave-----------%
qd1=0.5*sin(t);
qd2=0.5*sin(t);
qd3=0.5*sin(t);
% qd1 = 0.5*square(t);
% qd2 = 0.5*square(t);
% qd3 = 0.5*square(t);
qd = [qd1, qd2, qd3];

% Define marker indices (spread markers across time)
markerIdx = 1:50:length(t);

% Figure 1
% figure(1)
% plot(t, qd(:,1), 'k:', 'linewidth', 1);  % Ideal
% hold on
% plot(t, q(:,1), '-', 'Color', [1 0 0], 'LineWidth', 1, 'Marker', 'o', 'MarkerIndices', markerIdx);      % Red
% plot(t, qa(:,1), '--', 'Color', [0 1 0], 'LineWidth', 1, 'Marker', 's', 'MarkerIndices', markerIdx);     % Green
% plot(t, qs(:,1), '-.', 'Color', [0 0 1], 'LineWidth', 1, 'Marker', '^', 'MarkerIndices', markerIdx);     % Blue
% ylim([-1 1]);
% xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('Angle Tracking [rad]', 'FontSize', 12, 'FontWeight', 'bold');
% set(gca, 'LineWidth', 1.5, 'FontWeight', 'bold', 'FontSize', 18);
% legend('Ideal Angle for Link 1', 'Proposed Controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);

% Figure 1
figure(1)
markerIdx = 1000:1000:length(t);    % markers at every 100th data point

% Ideal reference (no markers)
plot(t, qd(:,1), 'k:', 'LineWidth', 2);  
hold on
plot(t, q(:,1), ...
     '-', ...
     'Color',[1 0 0], ...
     'LineWidth',1, ...
     'Marker','o', ...
     'MarkerSize',6, ...
     'MarkerIndices',markerIdx);
plot(t, qa(:,1), ...
     '--', ...
     'Color',[0 1 0], ...
     'LineWidth',1, ...
     'Marker','s', ...
     'MarkerSize',6, ...
     'MarkerIndices',markerIdx);
plot(t, qs(:,1), ...
     '-.', ...
     'Color',[0 0 1], ...
     'LineWidth',1, ...
     'Marker','^', ...
     'MarkerSize',6, ...
     'MarkerIndices',markerIdx);
% plot(t, q(:,1), ...
%      '-', ...
%      'Color',[1 0 0], ...
%      'LineWidth',2, ...
%      'Marker','o', ...
%      'MarkerSize',6, ...
%      'MarkerIndices',markerIdx);
% plot(t, qa(:,1), ...
%      '--', ...
%      'Color',[0 1 0], ...
%      'LineWidth',2, ...
%      'Marker','s', ...
%      'MarkerSize',6, ...
%      'MarkerIndices',markerIdx);
% plot(t, qs(:,1), ...
%      '-.', ...
%      'Color',[0 0 1], ...
%      'LineWidth',2, ...
%      'Marker','^', ...
%      'MarkerSize',6, ...
%      'MarkerIndices',markerIdx);
ylim([-1 1]);
xlabel('Time [sec]',   'FontSize',12,'FontWeight','bold');
ylabel('Angle Tracking [rad]', 'FontSize',12,'FontWeight','bold');
set(gca, 'LineWidth',1.5,'FontWeight','bold','FontSize',18);

legend('Ideal Angle for Link 1', ...
       'Proposed Controller', ...
       'Fuzzy-PID-SMC', ...
       'SMC-NN', ...
       'FontSize',10, 'Location','Best');


% Define once for all figures
markerIdx = 1000:1000:length(t);    % markers at every 1000th data point

% Figure 2
figure(2)
plot(t, qd(:,2), 'k:', 'LineWidth',2);  
hold on
plot(t, q(:,2), ...
     '-', ...
     'Color',[1 0 0], ...
     'LineWidth',1, ...
     'Marker','o', ...
     'MarkerSize',6, ...
     'MarkerIndices',markerIdx);
plot(t, qa(:,2), ...
     '--', ...
     'Color',[0 1 0], ...
     'LineWidth',1, ...
     'Marker','s', ...
     'MarkerSize',6, ...
     'MarkerIndices',markerIdx);
plot(t, qs(:,2), ...
     '-.', ...
     'Color',[0 0 1], ...
     'LineWidth',1, ...
     'Marker','^', ...
     'MarkerSize',6, ...
     'MarkerIndices',markerIdx);
% plot(t, q(:,2), ...
%      '-', ...
%      'Color',[1 0 0], ...
%      'LineWidth',2, ...
%      'Marker','o', ...
%      'MarkerSize',6, ...
%      'MarkerIndices',markerIdx);
% plot(t, qa(:,2), ...
%      '--', ...
%      'Color',[0 1 0], ...
%      'LineWidth',2, ...
%      'Marker','s', ...
%      'MarkerSize',6, ...
%      'MarkerIndices',markerIdx);
% plot(t, qs(:,2), ...
%      '-.', ...
%      'Color',[0 0 1], ...
%      'LineWidth',2, ...
%      'Marker','^', ...
%      'MarkerSize',6, ...
%      'MarkerIndices',markerIdx);
ylim([-1 1]);
xlabel('Time [sec]', 'FontSize',12,'FontWeight','bold');
ylabel('Angle Tracking [rad]', 'FontSize',12,'FontWeight','bold');
set(gca, 'LineWidth',1.5,'FontWeight','bold','FontSize',18);
legend('Ideal Angle for Link 2', ...
       'Proposed Controller', ...
       'Fuzzy-PID-SMC', ...
       'SMC-NN', ...
       'FontSize',10, 'Location','Best');


% Figure 3
figure(3)
plot(t, qd(:,3), 'k:', 'LineWidth',2);  
hold on
plot(t, q(:,3), ...
     '-', ...
     'Color',[1 0 0], ...
     'LineWidth',1, ...
     'Marker','o', ...
     'MarkerSize',6, ...
     'MarkerIndices',markerIdx);
plot(t, qa(:,3), ...
     '--', ...
     'Color',[0 1 0], ...
     'LineWidth',1, ...
     'Marker','s', ...
     'MarkerSize',6, ...
     'MarkerIndices',markerIdx);
plot(t, qs(:,3), ...
     '-.', ...
     'Color',[0 0 1], ...
     'LineWidth',1, ...
     'Marker','^', ...
     'MarkerSize',6, ...
     'MarkerIndices',markerIdx);
% plot(t, q(:,3), ...
%      '-', ...
%      'Color',[1 0 0], ...
%      'LineWidth',2, ...
%      'Marker','o', ...
%      'MarkerSize',6, ...
%      'MarkerIndices',markerIdx);
% plot(t, qa(:,3), ...
%      '--', ...
%      'Color',[0 1 0], ...
%      'LineWidth',2, ...
%      'Marker','s', ...
%      'MarkerSize',6, ...
%      'MarkerIndices',markerIdx);
% plot(t, qs(:,3), ...
%      '-.', ...
%      'Color',[0 0 1], ...
%      'LineWidth',2, ...
%      'Marker','^', ...
%      'MarkerSize',6, ...
%      'MarkerIndices',markerIdx);
ylim([-1 1]);
xlim([0 10]);
xlabel('Time [sec]', 'FontSize',12,'FontWeight','bold');
ylabel('Angle Tracking [rad]', 'FontSize',12,'FontWeight','bold');
set(gca, 'LineWidth',1.5,'FontWeight','bold','FontSize',18);
legend('Ideal Angle for Link 3', ...
       'Proposed Controller', ...
       'Fuzzy-PID-SMC', ...
       'SMC-NN', ...
       'FontSize',10, 'Location','Best');

% Figure 4
figure(4)
plot(t, e1, '-', 'Color', [1 0 0], 'linewidth', 2);  % Red
hold on
plot(t, ea1, '--', 'Color', [0 1 0], 'linewidth', 2);  % Green
plot(t, es1, ':', 'Color', [0 0 1], 'linewidth', 2);  % Blue
ylim([-1.5 1.5]);
% xlim([0 10]);
xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Tracking Error [rad]', 'FontSize', 10, 'FontWeight', 'bold');
set(gca, 'LineWidth', 1, 'FontWeight', 'bold', 'FontSize', 16);
legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);

% Figure 5
figure(5)
plot(t, e2, '-', 'Color', [1 0 0], 'linewidth', 2);
hold on
plot(t, ea2, '--', 'Color', [0 1 0], 'linewidth', 2);
plot(t, es2, '-.', 'Color', [0 0 1], 'linewidth', 2);
xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Tracking Error [rad]', 'FontSize', 10, 'FontWeight', 'bold');
set(gca, 'LineWidth', 1, 'FontWeight', 'bold', 'FontSize', 16);
legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);

% Figure 6
figure(6)
plot(t, e3, '-', 'Color', [1 0 0], 'linewidth', 2);
hold on
plot(t, ea3, '--', 'Color', [0 1 0], 'linewidth', 2);
plot(t, es3, '-.', 'Color', [0 0 1], 'linewidth', 2);
xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Tracking Error [rad]', 'FontSize', 10, 'FontWeight', 'bold');
set(gca, 'LineWidth', 1, 'FontWeight', 'bold', 'FontSize', 16);
legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);

% % Figure for u1
% figure(7)
% plot(t, 20*u1, '-', 'Color', [1 0 0], 'linewidth', 2); hold on
% plot(t, ua1/15, '--', 'Color', [0 1 0], 'linewidth', 2);
% plot(t, 10*us1, '-.', 'Color', [0 0 1], 'linewidth', 2);
% xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
% ylabel('u_1 [Nm]', 'FontSize', 10, 'FontWeight', 'bold');
% % title('Control Input u_1', 'FontSize', 12, 'FontWeight', 'bold');
% set(gca, 'LineWidth', 1, 'FontWeight', 'bold', 'FontSize', 16);
% legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);
% 
% % Figure for u2
% figure(8)
% plot(t, u2, '-', 'Color', [1 0 0], 'linewidth', 2); hold on
% plot(t, ua2/15, '--', 'Color', [0 1 0], 'linewidth', 2);
% plot(t, us2/3, '-.', 'Color', [0 0 1], 'linewidth', 2);
% xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
% ylabel('u_2 [Nm]', 'FontSize', 10, 'FontWeight', 'bold');
% % title('Control Input u_2', 'FontSize', 12, 'FontWeight', 'bold');
% set(gca, 'LineWidth', 1, 'FontWeight', 'bold', 'FontSize', 16);
% legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);
% 
% % Figure for u3
% figure(9)
% plot(t, 15*u3, '-', 'Color', [1 0 0], 'linewidth', 2); hold on
% plot(t, ua3/10, '--', 'Color', [0 1 0], 'linewidth', 2);
% plot(t, us3, '-.', 'Color', [0 0 1], 'linewidth', 2);
% xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
% ylabel('u_3 [Nm]', 'FontSize', 10, 'FontWeight', 'bold');
% % title('Control Input u_3', 'FontSize', 12, 'FontWeight', 'bold');
% set(gca, 'LineWidth', 1, 'FontWeight', 'bold', 'FontSize', 16);
% legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);



% Figure for u1
figure(7)
plot(t, 10*u1, '-', 'Color', [1 0 0], 'linewidth', 2); hold on
plot(t, ua1/100, '--', 'Color', [0 1 0], 'linewidth', 2);
plot(t, 2*us1, '-.', 'Color', [0 0 1], 'linewidth', 2);
xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('u_1 [Nm]', 'FontSize', 10, 'FontWeight', 'bold');
% title('Control Input u_1', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'LineWidth', 1, 'FontWeight', 'bold', 'FontSize', 16);
legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);

% Figure for u2
figure(8)
plot(t, u2, '-', 'Color', [1 0 0], 'linewidth', 2); hold on
plot(t, ua2/100, '--', 'Color', [0 1 0], 'linewidth', 2);
plot(t, us2, '-.', 'Color', [0 0 1], 'linewidth', 2);
xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('u_2 [Nm]', 'FontSize', 10, 'FontWeight', 'bold');
% title('Control Input u_2', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'LineWidth', 1, 'FontWeight', 'bold', 'FontSize', 16);
legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);

% Figure for u3
figure(9)
plot(t, 10*u3, '-', 'Color', [1 0 0], 'linewidth', 2); hold on
plot(t, ua3/100, '--', 'Color', [0 1 0], 'linewidth', 2);
plot(t, us3, '-.', 'Color', [0 0 1], 'linewidth', 2);
xlabel('Time [sec]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('u_3 [Nm]', 'FontSize', 10, 'FontWeight', 'bold');
% title('Control Input u_3', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'LineWidth', 1, 'FontWeight', 'bold', 'FontSize', 16);
legend('Proposed controller','Fuzzy-PID-SMC','SMC-NN','FontSize', 10);



