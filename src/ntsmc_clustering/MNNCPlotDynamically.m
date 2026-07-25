% Load the data (e1, e2, e3, de1, de2, de3)
% Assumes these variables are already in the workspace

% Define regions for initial, mid, and final (1:1000, 25000:26000, 49000:50000)
regions = {1:1000, 25000:26000, 49000:50000};
region_names = {'Initial', 'Mid', 'Final'};

% Initialize cell arrays to store clusters and data for each region and dataset
all_clusters = cell(3, 3); % 3 datasets x 3 regions
all_data = cell(3, 3); % To store data matrices for easy plotting
all_centroids = cell(3, 3); % To store centroids for each dataset and region

% Define datasets for easy iteration
e_datasets = {e1, e2, e3};
de_datasets = {de1, de2, de3};

% Parameters for MNNC clustering (adjust H and Nmin as needed)
H = 5; % Distance level (example value)
Nmin = 20; % Minimum number of samples for each cluster (example value)

% Perform clustering on each dataset and region
for d = 1:3 % Iterate over the datasets (e1, e2, e3)
    for r = 1:3 % Iterate over the regions (initial, mid, final)
        % Extract current dataset and region
        e_current = e_datasets{d}(regions{r});
        de_current = de_datasets{d}(regions{r});
        
        % Create the data matrix for the current region
        data = [e_current'; de_current'];
        all_data{d, r} = data; % Store the data for plotting
        
        % Perform clustering using MNNC (assume MNNC is defined)
        clusters = MNNC(data, H, Nmin);
        all_clusters{d, r} = clusters; % Store the clusters
        
        % Calculate centroids for the current clusters
        centroids = zeros(2, length(clusters));
        for i = 1:length(clusters)
            if ~isempty(clusters{i})
                % Get the indices for the current cluster
                cluster_indices = clusters{i};
                
                % Extract the data points for the current cluster
                cluster_data = data(:, cluster_indices);
                
                % Calculate the centroid as the mean of the cluster data points
                centroids(:, i) = mean(cluster_data, 2);
            end
        end
        all_centroids{d, r} = centroids; % Store the centroids
        
        % Plot the current dataset and region
        figure;
        hold on;
        
        % Define colors and markers for dynamic plotting
        colors = lines(length(clusters)); % Generate colors for the current number of clusters
        markers = {'o', 's', 'd', '^', 'v', '>', '<', 'p', 'h', '*'}; % Extendable marker types
        
        % Plot clusters for the current dataset and region
        for i = 1:length(clusters)
            if ~isempty(clusters{i})
                cluster_indices = clusters{i};
                scatter(data(1, cluster_indices), data(2, cluster_indices), 50, ...
                    'Marker', markers{mod(i-1, length(markers)) + 1}, ...
                    'MarkerFaceColor', colors(i, :), ...
                    'DisplayName', ['Cluster ' num2str(i)]);
            end
        end
        
        % Plot the centroids for the current clusters
        for i = 1:length(clusters)
            if ~isempty(centroids(:, i))
                scatter(centroids(1, i), centroids(2, i), 100, 'k', 'filled', 'p', ...
                    'DisplayName', ['Centroid ' num2str(i)]);
            end
        end
        
        % Customize the plot
        xlabel('X-axis');
        ylabel('Y-axis');
        title(['Dataset ' num2str(d) ' - ' region_names{r} ' Region']);
        legend show;
        hold off;
    end
end


% ---------------------------------------------------

% % Load the data (e1, e2, e3, de1, de2, de3)
% % Assumes these variables are already in the workspace
% 
% % Define regions for initial, mid, and final (1:1000, 25000:26000, 49000:50000)
% regions = {1:1000, 25000:26000, 49000:50000};
% region_names = {'Initial', 'Mid', 'Final'};
% 
% % Initialize cell arrays to store clusters for each region and dataset
% all_clusters = cell(3, 3); % 3 datasets x 3 regions
% all_data = cell(3, 3); % To store data matrices for easy plotting
% 
% % Define datasets for easy iteration
% e_datasets = {e1, e2, e3};
% de_datasets = {de1, de2, de3};
% 
% % Parameters for MNNC clustering (adjust H and Nmin as needed)
% H = 5; % Distance level (example value)
% Nmin = 20; % Minimum number of samples for each cluster (example value)
% 
% % Perform clustering on each dataset and region
% for d = 1:3 % Iterate over the datasets (e1, e2, e3)
%     for r = 1:3 % Iterate over the regions (initial, mid, final)
%         % Extract current dataset and region
%         e_current = e_datasets{d}(regions{r});
%         de_current = de_datasets{d}(regions{r});
% 
%         % Create the data matrix for the current region
%         data = [e_current'; de_current'];
%         all_data{d, r} = data; % Store the data for plotting
% 
%         % Perform clustering using MNNC (assume MNNC is defined)
%         clusters = MNNC(data, H, Nmin);
%         all_clusters{d, r} = clusters; % Store the clusters
% 
%         % Plot the current dataset and region
%         figure;
%         hold on;
% 
%         % Define colors and markers for dynamic plotting
%         colors = lines(length(clusters)); % Generate colors for the current number of clusters
%         markers = {'o', 's', 'd', '^', 'v', '>', '<', 'p', 'h', '*'}; % Extendable marker types
% 
%         % Plot clusters for the current dataset and region
%         for i = 1:length(clusters)
%             if ~isempty(clusters{i})
%                 cluster_indices = clusters{i};
%                 scatter(data(1, cluster_indices), data(2, cluster_indices), 50, ...
%                     'Marker', markers{mod(i-1, length(markers)) + 1}, ...
%                     'MarkerFaceColor', colors(i, :), ...
%                     'DisplayName', ['Cluster ' num2str(i)]);
%             end
%         end
% 
%         % Customize the plot
%         xlabel('X-axis');
%         ylabel('Y-axis');
%         title(['Dataset ' num2str(d) ' - ' region_names{r} ' Region']);
%         legend show;
%         hold off;
%     end
% end


% --------------------------------------------------------


% % Load the data (e1, e2, e3, de1, de2, de3)
% % Assumes these variables are already in the workspace
% 
% % Define regions for initial, mid, and final (1:1000, 25000:26000, 49000:50000)
% regions = {1:1000, 25000:26000, 49000:50000};
% region_names = {'Initial', 'Mid', 'Final'};
% 
% % Initialize cell arrays to store clusters for each region and dataset
% all_clusters = cell(3, 3); % 3 datasets x 3 regions
% all_data = cell(3, 3); % To store data matrices for easy plotting
% 
% % Define datasets for easy iteration
% e_datasets = {e1, e2, e3};
% de_datasets = {de1, de2, de3};
% 
% % Parameters for MNNC clustering (adjust H and Nmin as needed)
% H = 5; % Distance level (example value)
% Nmin = 20; % Minimum number of samples for each cluster (example value)
% 
% % Perform clustering on each dataset and region
% for d = 1:3 % Iterate over the datasets (e1, e2, e3)
%     for r = 1:3 % Iterate over the regions (initial, mid, final)
%         % Extract current dataset and region
%         e_current = e_datasets{d}(regions{r});
%         de_current = de_datasets{d}(regions{r});
% 
%         % Create the data matrix for the current region
%         data = [e_current'; de_current'];
%         all_data{d, r} = data; % Store the data for plotting
% 
%         % Perform clustering using MNNC (assume MNNC is defined)
%         clusters = MNNC(data, H, Nmin);
%         all_clusters{d, r} = clusters; % Store the clusters
%     end
% end
% 
% % Plot all clusters dynamically for each dataset and region
% figure;
% hold on;
% 
% % Define colors and markers for dynamic plotting
% colors = lines(9); % Generate colors for 9 clusters
% markers = {'o', 's', 'd', '^', 'v', '>', '<', 'p', 'h', '*'}; % Extendable marker types
% 
% % Loop to plot all clusters
% for d = 1:3 % Iterate over datasets
%     for r = 1:3 % Iterate over regions
%         clusters = all_clusters{d, r};
%         data = all_data{d, r};
% 
%         % Plot clusters
%         for i = 1:length(clusters)
%             if ~isempty(clusters{i})
%                 cluster_indices = clusters{i};
%                 scatter(data(1, cluster_indices), data(2, cluster_indices), 50, ...
%                     'Marker', markers{mod(i-1, length(markers)) + 1}, ...
%                     'MarkerFaceColor', colors(mod((d-1)*3 + r - 1, size(colors, 1)) + 1, :), ...
%                     'DisplayName', ['Dataset ' num2str(d) ' ' region_names{r} ' Cluster ' num2str(i)]);
%             end
%         end
%     end
% end
% 
% % Customize the plot
% xlabel('X-axis');
% ylabel('Y-axis');
% title('Plot of Clusters from 3 Datasets and 3 Regions');
% legend show;
% hold off;