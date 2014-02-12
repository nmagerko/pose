% load point cloud into variable
load ../output/cocoa_puffs.mat cloud;

% find the threshold by finding the maximum distance between any two points,
% and taking five percent of this value
t = max(pdist(cloud)) * 0.005;

iterations = 0
global current_outliers = cloud
while (iterations < 5)
	% get the first set of inliers using the transpose of the pointcloud and
	% the above threshold
	[M, ~, inliers] = ransacfitplane(transpose(current_outliers), t);
	
	% get the outliers
	v = false(size(cloud, 1), 1);
	v(inliers) = 1;
	outliers = find(~v);
	
	% get all of the inliers as points from the point cloud and do the same for
	% the outlier points
	inlier_points = cloud(inliers, :);
	outlier_points = cloud(outliers, :);

	[x, y] = meshgrid([min(cloud(inliers, 1)), max(cloud(inliers, 1))],[min(cloud(inliers, 2)), max(cloud(inliers, 2))])

	% M - 4x1 array of plane coefficients in the form
	% b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0 [from ransacfitplane]
	z = -(x*M(1)+y*M(2)+M(4))/M(3)

	mesh(x, y, z)
	hold on;
	% uncomment to plot inliers on plane
	% plot3(cloud(inliers, 1), cloud(inliers, 2), cloud(inliers, 3), 'xr');
	current_outilers = outlier_points 
	iterations = iterations + 1
endwhile
hold off;
