% Helper function to calculate total distance to centroid for a cluster
function total_distance = calculate_total_distance(cluster_indices, data)
    cluster_points = data(:, cluster_indices);
    centroid = mean(cluster_points, 2); % Calculate the centroid of the cluster
    total_distance = sum(sqrt(sum((cluster_points - centroid).^2, 1))); % Sum of distances to centroid
end