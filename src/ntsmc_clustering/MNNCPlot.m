% Assume 'data' is your dataset with a size of [2, N] (where N is the number of data points)
% and 'clusters' is your cell array containing indices for any number of clusters.

% Get the number of clusters dynamically
num_clusters = length(clusters);

% Initialize an array to store the centroids of each cluster
centroids = zeros(2, num_clusters);

% Loop through each cluster and calculate the centroid
for i = 1:num_clusters
    if ~isempty(clusters{i})
        % Get the indices for the current cluster
        cluster_indices = clusters{i};
        
        % Extract the data points for the current cluster
        cluster_data = data(:, cluster_indices);
        
        % Calculate the centroid as the mean of the cluster data points
        centroids(:, i) = mean(cluster_data, 2);
    end
end

% Plot the clusters with different colors and markers
figure;
hold on;

% Define colors and markers for the clusters
colors = lines(num_clusters); % Generate distinct colors for each cluster
markers = {'o', 's', 'd', '^', 'v', '>', '<', 'p', 'h', '*'}; % Extendable marker types

% Loop to plot each cluster with a unique color and marker
for i = 1:num_clusters
    if ~isempty(clusters{i})
        cluster_indices = clusters{i};
        scatter(data(1, cluster_indices), data(2, cluster_indices), 50, ...
            'Marker', markers{mod(i-1, length(markers)) + 1}, ...
            'MarkerFaceColor', colors(i, :), ...
            'DisplayName', ['Cluster ' num2str(i)]);
    end
end

% Plot the centroids with a distinct marker (e.g., black stars) but exclude them from the legend
for i = 1:num_clusters
    if ~isempty(centroids(:, i))
        scatter(centroids(1, i), centroids(2, i), 100, 'k', 'filled', 'p', ...
            'HandleVisibility', 'off'); % Exclude centroids from legend
    end
end

% Customize the plot
% Make x and y axes bold and increase font size
set(gca, 'LineWidth', 1);
set(gca, 'FontWeight', 'bold');
set(gca, 'FontSize', 18);
xlabel('X-axis');
ylabel('Y-axis');
title(['Plot of ' num2str(num_clusters) ' clusters with centroid points']);
legend show;
hold off;

