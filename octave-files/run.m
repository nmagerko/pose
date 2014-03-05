% load point cloud into variable
load ../output/bunny-door.mat cloud;

% find the threshold by finding the maximum distance between any two points,
% and taking five percent of this value
t = max(pdist(cloud))*0.05;

current_outliers = cloud;
axis off
hold on
while (true)
	% get the first set of inliers indicies using the transpose of the pointcloud and
	% the above threshold (if there are enough points)
	try
		[M, ~, inliers] = ransacfitplane(transpose(current_outliers), t);
	
		% get the new set of outlier indicies
		v = false(size(current_outliers, 1), 1);
		v(inliers) = 1;
		outliers = find(~v);
	
		% get all of the inliers as points from the point cloud and do the same for
		% the outliers
		inlier_points = cloud(inliers, :);
		outlier_points = cloud(outliers, :);

		% solve for the x and y components of the plane
		[x, y] = meshgrid([min(cloud(inliers, 1)), max(cloud(inliers, 1))],[min(cloud(inliers, 2)), max(cloud(inliers, 2))]);

		% M is a 4x1 array of plane coefficients in the form
		% b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0 [from ransacfitplane]
		% solve for the z component
		z = -(x*M(1)+y*M(2)+M(4))/M(3);

		% plot the x, y, and z components as a mesh
		mesh(x, y, z);
		% plot the current inliers (red x's), along with the current outliers (blue o's)
		plot3(cloud(inliers, 1), cloud(inliers, 2), cloud(inliers, 3), 'xr');
		plot3(cloud(outliers, 1), cloud(outliers, 2), cloud(outliers, 3), 'ob');
		% set the current outlier points as the input for the next iteration
		current_outliers = outlier_points;
		pause
	catch
		% if ransacfitplane doesn't have any more points to fit, exit
		if (strcmp(lasterror.message, "too few points to fit plane"))
			break;
		endif
	end
endwhile
hold off;
