%%%%%%%%%%%% CONST + SQUARE WAVE %%%%%%%%%%%%

% % Define the threshold
% threshold = 1e-3; 
% start_time = 0.005; % Start evaluating for t > 0.005 seconds
% 
% % Filter indices where t > 0.005
% valid_indices = find(t > start_time);
% 
% % Initialize arrays to store convergence times for each link and method
% convergence_times_ntsmc = [NaN, NaN, NaN]; % For e1, e2, e3
% convergence_times_fpid = [NaN, NaN, NaN];  % For ea1, ea2, ea3
% 
% % --- NTSMC Convergence Times ---
% % Find the first index where abs(error) < threshold for each link
% id_case1 = find(abs(e1(valid_indices)) < threshold, 1); % Link 1
% if ~isempty(id_case1)
%     convergence_times_ntsmc(1) = t(valid_indices(id_case1));
% end
% 
% id_case2 = find(abs(e2(valid_indices)) < threshold, 1); % Link 2
% if ~isempty(id_case2)
%     convergence_times_ntsmc(2) = t(valid_indices(id_case2));
% end
% 
% id_case3 = find(abs(e3(valid_indices)) < threshold, 1); % Link 3
% if ~isempty(id_case3)
%     convergence_times_ntsmc(3) = t(valid_indices(id_case3));
% end
% 
% % --- Fuzzy PID SMC Convergence Times ---
% ida_case1 = find(abs(ea1(valid_indices)) < threshold, 1); % Link 1
% if ~isempty(ida_case1)
%     convergence_times_fpid(1) = t(valid_indices(ida_case1));
% end
% 
% ida_case2 = find(abs(ea2(valid_indices)) < threshold, 1); % Link 2
% if ~isempty(ida_case2)
%     convergence_times_fpid(2) = t(valid_indices(ida_case2));
% end
% 
% ida_case3 = find(abs(ea3(valid_indices)) < threshold, 1); % Link 3
% if ~isempty(ida_case3)
%     convergence_times_fpid(3) = t(valid_indices(ida_case3));
% end
% 
% % Print Results
% fprintf('--- Convergence Times for NTSMC ---\n');
% fprintf('Link 1: %.6f s\n', convergence_times_ntsmc(1));
% fprintf('Link 2: %.6f s\n', convergence_times_ntsmc(2));
% fprintf('Link 3: %.6f s\n', convergence_times_ntsmc(3));
% 
% fprintf('\n--- Convergence Times for Fuzzy PID SMC ---\n');
% fprintf('Link 1: %.6f s\n', convergence_times_fpid(1));
% fprintf('Link 2: %.6f s\n', convergence_times_fpid(2));
% fprintf('Link 3: %.6f s\n', convergence_times_fpid(3));







% %%%%%%%%%%%% CONST + SQUARE WAVE %%%%%%%%%%%%
% Define the threshold
threshold = 1e-3; 
start_time = 0.005; % Start evaluating for t > 0.005 seconds

% Filter indices where t > 0.005
valid_indices = find(t > start_time);

% Initialize arrays to store convergence times for each link and method
convergence_times_ntsmc = [NaN, NaN, NaN]; % For e1, e2, e3
convergence_times_fpid  = [NaN, NaN, NaN]; % For ea1, ea2, ea3
convergence_times_smcnn = [NaN, NaN, NaN]; % For es1, es2, es3

% --- NTSMC Convergence Times ---
for link = 1:3
    err = eval(sprintf('e%d', link));           % e1, e2, e3
    idx = find(abs(err(valid_indices)) < threshold, 1);
    if ~isempty(idx)
        convergence_times_ntsmc(link) = t(valid_indices(idx));
    end
end

% --- Fuzzy PID SMC Convergence Times ---
for link = 1:3
    err = eval(sprintf('ea%d', link));          % ea1, ea2, ea3
    idx = find(abs(err(valid_indices)) < threshold, 1);
    if ~isempty(idx)
        convergence_times_fpid(link) = t(valid_indices(idx));
    end
end

% --- SMC‑NN Convergence Times ---
for link = 1:3
    err = eval(sprintf('es%d', link));          % es1, es2, es3
    idx = find(abs(err(valid_indices)) < threshold, 1);
    if ~isempty(idx)
        convergence_times_smcnn(link) = t(valid_indices(idx));
    end
end

% Print Results
fprintf('--- Convergence Times for NTSMC ---\n');
for link = 1:3
    fprintf('Link %d: %.6f s\n', link, convergence_times_ntsmc(link));
end

fprintf('\n--- Convergence Times for Fuzzy PID SMC ---\n');
for link = 1:3
    fprintf('Link %d: %.6f s\n', link, convergence_times_fpid(link));
end

fprintf('\n--- Convergence Times for SMC‑NN ---\n');
for link = 1:3
    fprintf('Link %d: %.6f s\n', link, convergence_times_smcnn(link));
end
