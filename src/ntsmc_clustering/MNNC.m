function clusters = MNNC(data, H, Nmin)
% MNNC: Modified Nearest Neighbor-based Clustering algorithm for training RBFNN
% Input: 
%   data - Input data in 2xN matrix format
%   H - distance level number that is determined by user.
%   Nmin - Minimum number of samples for each cluster
% Output: 
%   clusters - A cell array where each cell contains the indices of samples in one cluster

    % Step 1: Sort the data by the first and second components separately
    [~, idx1] = sort(data(1, :));
    sorted_data_x = data(:, idx1); % Sort data by the first component

    [~, idx2] = sort(data(2, :));
    sorted_data_y = data(:, idx2); % Sort data by the first component

    n = size(data, 2); % Total number of samples

    % Step 2: Calculate distance attribute for each sample based on the two components
    di0 = zeros(n, 1); % Each sample will have its own distance attribute di0

    for i = 2:n-1 % Not consider the first and last samples as they do not have both left and right neighbors
        % Sorted by first component
        dx1 = norm(sorted_data_x(:, i) - sorted_data_x(:, i-1));
        dx2 = norm(sorted_data_x(:, i) - sorted_data_x(:, i+1));
        % Sorted by second component
        dy1 = norm(sorted_data_y(:, i) - sorted_data_y(:, i-1));
        dy2 = norm(sorted_data_y(:, i) - sorted_data_y(:, i+1));

        % Calculate di0 as the minimum of the four distances
        di0(i) = min([dx1, dx2, dy1, dy2]);
        % disp(['di0=',num2str(di0(i))])
    end

    % Step 3: Calculate distance attribute for each sample
    di = inf(n, 1);

    if n > 1  % should be n>2?
        for i = 2:n-1 % Not consider the first and last samples as they do not have both left and right neighbors
            % Get current sample coordinates
            xi = data(1, i);
            yi = data(2, i);

            % Consider the previous and next samples only
            for j = [i-1, i+1]
                xj = data(1, j);
                yj = data(2, j);
                % Calculate Euclidean distance between Pi and Pj
                dj = sqrt((xj - xi)^2 + (yj - yi)^2);

                % Update di to be the minimum distance that satisfies the condition |xj - xi| <= di and |yj - yi| <= di
                di(i) = min(di(i), dj);
            end
            % disp(['di(i)=',num2str(di(i))])
        end

        % For the first sample, only consider the next sample
        xj = data(1, 2);
        yj = data(2, 2);
        dj = sqrt((xj - data(1, 1))^2 + (yj - data(2, 1))^2);
        di(1) = min(di(1), dj);
        % disp(['di(1)=',num2str(di(1))])

        % For the last sample, only consider the previous sample
        xj = data(1, n-1);
        yj = data(2, n-1);
        dj = sqrt((xj - data(1, n))^2 + (yj - data(2, n))^2);
        di(n) = min(di(n), dj);
        % disp(['di(n)=',num2str(di(n))])
    end

    % Step 4: Set distance step
    dmin = min(di);
    dmax = max(di);
    dstep = (dmax + dmin) / H;
    % disp(['dstep=',num2str(dstep)])

    % Step 5: For each distance level, find the nearest neighbors and establish clusters
    clusters = cell(n, 1); % Initialize clusters for each sample
    for h = 1:H
        threshold = dmin + h * dstep;
        % disp(['threshold=',num2str(threshold)])
        for i = 1:n
            cluster = i; % Start with the current sample index 
            % Find neighbors within the threshold distance
            for j = 1:n
                if i ~= j
                    distance = norm(data(:, j) - data(:, i)); % Euclidean distance between sample i and sample j
                    % disp(['distance=',num2str(distance)])
                    if distance <= threshold
                        cluster = [cluster, j]; % Add neighbor sample index
                    end
                end
            end
            % Update cluster for the current sample
            clusters{i} = unique([clusters{i}, cluster]); % Update cluster with sample indices
            disp(['clusters=',num2str(clusters{i})])
        end
    end

<<<<<<< HEAD

=======
>>>>>>> 74fd74ee2e40386f4fe8e0f3145f9dbfabcc9fff
    % Step 6: Merge clusters only if the centroid distance is below a specified threshold
    % merge_threshold = 0.0285; % Define a threshold for merging clusters
    merge_threshold = 0.20;

    for i = 1:n
        for j = i+1:n
            if ~isempty(clusters{i}) && ~isempty(clusters{j})
                % Check if they share samples
                if ~isempty(intersect(clusters{i}, clusters{j}))
                    % Calculate centroids of the two clusters
                    centroid_i = mean(data(:, clusters{i}), 2);
                    centroid_j = mean(data(:, clusters{j}), 2);
                    % Calculate Euclidean distance between centroids
                    centroid_dist = norm(centroid_i - centroid_j);

                    % Merge only if the distance is below the threshold
                    if centroid_dist < merge_threshold
                        clusters{i} = unique([clusters{i}, clusters{j}]);
                        clusters{j} = [];
                    end
                end
            end
        end
    end 
      

    % Step 7: Merge small clusters into the nearest cluster
    for i = 1:n
        if length(clusters{i}) < Nmin && ~isempty(clusters{i})
            % Find the nearest cluster to merge
            nearest_cluster_idx = -1;
            min_distance = inf;
            for j = 1:n
                if i ~= j && ~isempty(clusters{j})
                    % Calculate the distance between cluster i and cluster j
                    xi = mean(data(:, clusters{i}), 2);
                    xj = mean(data(:, clusters{j}), 2);
                    distance = norm(xj - xi);
                    if distance < min_distance
                        min_distance = distance;
                        nearest_cluster_idx = j;
                    end
                end
            end
            % Merge into the nearest cluster
            if nearest_cluster_idx ~= -1
                clusters{nearest_cluster_idx} = unique([clusters{nearest_cluster_idx}, clusters{i}]);
                clusters{i} = [];
            end
        end
    end

    % Step 8: Remove empty clusters
    clusters = clusters(~cellfun('isempty', clusters));

end
<<<<<<< HEAD
=======

>>>>>>> 74fd74ee2e40386f4fe8e0f3145f9dbfabcc9fff