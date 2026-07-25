% Calculate RMSE for each corresponding component in q and qd
rmse_q1_qd1 = rmse(q(:, 1), qd(:, 1));
rmse_q2_qd2 = rmse(q(:, 2), qd(:, 2));
rmse_q3_qd3 = rmse(q(:, 3), qd(:, 3));

% Display the RMSE values for q and qd
fprintf('RMSE for q1 and qd1: %.6f\n', rmse_q1_qd1);
fprintf('RMSE for q2 and qd2: %.6f\n', rmse_q2_qd2);
fprintf('RMSE for q3 and qd3: %.6f\n', rmse_q3_qd3);

% Calculate RMSE for each corresponding component
rmse_qa1_qda1 = rmse(qa(:, 1), qda(:, 1));
rmse_qa2_qda2 = rmse(qa(:, 2), qda(:, 2));
rmse_qa3_qda3 = rmse(qa(:, 3), qda(:, 3));

% Display the RMSE values
fprintf('RMSE for qa1 and qda1: %.6f\n', rmse_qa1_qda1);
fprintf('RMSE for qa2 and qda2: %.6f\n', rmse_qa2_qda2);
fprintf('RMSE for qa3 and qda3: %.6f\n', rmse_qa3_qda3);

% % Calculate RMSE for each corresponding component in qs and qds
rmse_qs1_qds1 = rmse(qs(:, 1), qds(:, 1));
rmse_qs2_qds2 = rmse(qs(:, 2), qds(:, 2));
rmse_qs3_qds3 = rmse(qs(:, 3), qds(:, 3));

% Display the RMSE values for qs and qds
fprintf('RMSE for qs1 and qds1: %.6f\n', rmse_qs1_qds1);
fprintf('RMSE for qs2 and qds2: %.6f\n', rmse_qs2_qds2);
fprintf('RMSE for qs3 and qds3: %.6f\n', rmse_qs3_qds3);
