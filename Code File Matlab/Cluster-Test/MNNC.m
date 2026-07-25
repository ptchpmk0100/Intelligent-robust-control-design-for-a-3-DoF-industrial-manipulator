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
                % if abs(xj - xi) <= di0(i) && abs(yj - yi) <= di0(i)
                %     di(i) = min(di(i), dj);
                % end
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

    % % Step 6: Merge clusters if they contain the same samples
    % for i = 1:n
    %     for j = i+1:n
    %         if ~isempty(clusters{i}) && ~isempty(clusters{j}) && ~isempty(intersect(clusters{i}, clusters{j}))
    %             clusters{i} = unique([clusters{i}, clusters{j}]);
    %             clusters{j} = [];
    %         end
    %     end
    % end

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

    % % Step 6: Merge clusters only if the centroid distance is below a specified threshold
    % merge_threshold = 0.0285; % Define a threshold for merging clusters
    % 
    % for i = 1:n
    %     for j = i+1:n
    %         if ~isempty(clusters{i}) && ~isempty(clusters{j})
    %             % Check if they share samples
    %             if ~isempty(intersect(clusters{i}, clusters{j}))
    %                 % Calculate centroids of the two clusters
    %                 centroid_i = mean(data(:, clusters{i}), 2);
    %                 centroid_j = mean(data(:, clusters{j}), 2);
    %                 % Calculate Euclidean distance between centroids
    %                 centroid_dist = norm(centroid_i - centroid_j);
    % 
    %                 % Merge only if the distance is below the threshold and clusters are continuous
    %                 if centroid_dist < merge_threshold && check_spatial_continuity([clusters{i}, clusters{j}], data)
    %                     clusters{i} = unique([clusters{i}, clusters{j}]);
    %                     clusters{j} = [];
    %                 end
    %             end
    %         end
    %     end
    % end  

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

% % Function to check if a cluster is spatially continuous
% function is_continuous = check_spatial_continuity(cluster_indices, data)
%     cluster_data = data(:, cluster_indices);
%     distance_matrix = pdist2(cluster_data', cluster_data'); % Calculate pairwise distances
% 
%     % Set a threshold for maximum allowed distance between any two points to consider continuity
%     max_gap = max(distance_matrix(:)) / 2; % Adjust this threshold as needed
% 
%     % Check if all distances are below the threshold
%     is_continuous = all(distance_matrix(:) <= max_gap);
% end
% 
% % Function to check if a cluster is spatially continuous with an adjustable threshold
% function is_continuous = check_spatial_continuity(cluster_indices, data)
%     cluster_data = data(:, cluster_indices);
%     distance_matrix = pdist2(cluster_data', cluster_data'); % Calculate pairwise distances
% 
%     % Set a wider threshold for maximum allowed distance between any two points to consider continuity
%     % For example, set this to a larger constant or a proportion of the data range
%     max_gap = 0.1; % Example wider threshold (adjust based on your dataset's scale)
% 
%     % Check if all distances are below the threshold (only consider the upper triangle to avoid redundancy)
%     is_continuous = all(distance_matrix(triu(true(size(distance_matrix)), 1)) <= max_gap);
% end




% -------------------------------
% function clusters = MNNC(data, H, Nmin)
%     n = size(data, 2); % Total number of samples
%     clusters = cell(n, 1); % Initialize clusters for each sample
% 
%     % Step 1: Pre-calculate distance matrix
%     dist_matrix = pdist2(data', data'); % Pairwise distances between all samples
% 
%     % Step 2: Initialize each sample as its own cluster
%     for i = 1:n
%         clusters{i} = i;
%     end
% 
%     % Step 3: Set distance step based on minimum and maximum distances
%     di = min(dist_matrix + diag(inf(n, 1))); % Minimum distance ignoring self-distances
%     dmin = min(di);
%     dmax = max(di);
%     dstep = (dmax - dmin) / H; % Define the step size for threshold
% 
%     % Step 4: For each distance level, group samples within the threshold distance
%     for h = 1:H
%         threshold = dmin + h * dstep;
%         for i = 1:n
%             for j = i+1:n
%                 if dist_matrix(i, j) <= threshold
%                     clusters{i} = unique([clusters{i}, clusters{j}]);
%                     clusters{j} = clusters{i}; % Synchronize clusters
%                 end
%             end
%         end
%     end
% 
%     % Step 5: Merge clusters with shared samples
%     % Ensure merging is done only once per pair of clusters to avoid redundancy
%     for i = 1:n
%         for j = i+1:n
%             if ~isempty(clusters{i}) && ~isempty(clusters{j}) && ~isempty(intersect(clusters{i}, clusters{j}))
%                 clusters{i} = unique([clusters{i}, clusters{j}]);
%                 clusters{j} = []; % Clear the second cluster to avoid repetitive merging
%             end
%         end
%     end
% 
%     % Step 6: Merge small clusters into the nearest cluster if they have fewer than Nmin samples
%     for i = 1:n
%         if length(clusters{i}) < Nmin && ~isempty(clusters{i})
%             % Find the nearest cluster to merge with
%             min_distance = inf;
%             nearest_cluster_idx = -1;
%             for j = 1:n
%                 if i ~= j && ~isempty(clusters{j})
%                     % Calculate the distance between cluster i and cluster j
%                     xi = mean(data(:, clusters{i}), 2);
%                     xj = mean(data(:, clusters{j}), 2);
%                     distance = norm(xj - xi);
%                     if distance < min_distance
%                         min_distance = distance;
%                         nearest_cluster_idx = j;
%                     end
%                 end
%             end
%             % Merge into the nearest cluster
%             if nearest_cluster_idx ~= -1
%                 clusters{nearest_cluster_idx} = unique([clusters{nearest_cluster_idx}, clusters{i}]);
%                 clusters{i} = []; % Clear the cluster to prevent re-merging
%             end
%         end
%     end
% 
%     % Step 7: Remove empty clusters
%     clusters = clusters(~cellfun('isempty', clusters));
% end

% function clusters = MNNC(data, H, Nmin)
%     % Step 1: Sort data to prepare for nearest neighbor clustering based on distance and continuity
%     n = size(data, 2); 
%     di0 = zeros(n, 1); % Initialize distance attribute for each sample
% 
%     % Step 2: Calculate minimum distance (di0) for each sample based on nearest neighbors
%     for i = 2:n-1
%         dx1 = norm(data(:, i) - data(:, i-1));
%         dx2 = norm(data(:, i) - data(:, i+1));
%         di0(i) = min([dx1, dx2]);
%     end
% 
%     % Step 3: Define distance levels based on di values
%     di_min = min(di0);
%     di_max = max(di0);
%     dstep = (di_max - di_min) / H;
% 
%     % Step 4: Initialize clusters based on distance threshold levels
%     clusters = cell(n, 1); % Initialize each sample as its own cluster
%     for h = 1:H
%         threshold = di_min + h * dstep;
%         for i = 1:n
%             cluster = i;
%             for j = i+1:n
%                 if i ~= j
%                     distance = norm(data(:, j) - data(:, i));
%                     if distance <= threshold
%                         cluster = [cluster, j];
%                     end
%                 end
%             end
%             clusters{i} = unique([clusters{i}, cluster]);
%         end
%     end
% 
%     % Step 5: Split clusters with non-continuous indices and merge small clusters
%     for i = 1:n
%         if ~isempty(clusters{i}) && length(clusters{i}) < Nmin
%             nearest_cluster = -1;
%             min_dist = inf;
%             for j = 1:n
%                 if i ~= j && ~isempty(clusters{j}) && length(clusters{j}) >= Nmin
%                     xi = mean(data(:, clusters{i}), 2);
%                     xj = mean(data(:, clusters{j}), 2);
%                     dist = norm(xj - xi);
%                     if dist < min_dist
%                         min_dist = dist;
%                         nearest_cluster = j;
%                     end
%                 end
%             end
%             if nearest_cluster ~= -1
%                 clusters{nearest_cluster} = unique([clusters{nearest_cluster}, clusters{i}]);
%                 clusters{i} = [];
%             end
%         end
%     end
% 
%     % Step 6: Remove empty cells and return final clusters
%     clusters = clusters(~cellfun('isempty', clusters));
% end










% Plot
% Plot each cluster with a unique color
% figure;
% hold on;
% colors = lines(length(clusters)); % Generate a set of colors
% 
% for i = 1:length(clusters)
%     % Get the indices of the points in the i-th cluster
%     cluster_indices = clusters{i};
% 
%     % Extract the data points for the current cluster
%     cluster_data = data(cluster_indices, :);
% 
%     % Plot the points for this cluster
%     scatter(cluster_data(:, 1), cluster_data(:, 2), 50, 'filled', ...
%         'MarkerFaceColor', colors(i, :), 'DisplayName', ['Cluster ' num2str(i)]);
% end
% 
% % Add legend and axis labels
% legend show;
% xlabel('X-axis');
% ylabel('Y-axis');
% title('Cluster Visualization');
% hold off;