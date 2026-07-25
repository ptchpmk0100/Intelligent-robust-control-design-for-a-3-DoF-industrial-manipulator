%%%%%%%%%%%% CONST + SQUARE WAVE %%%%%%%%%%%%

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
