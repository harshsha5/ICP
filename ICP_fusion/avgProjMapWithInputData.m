function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;

    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====
    
    %==================================================================================
    % Write your code here...
    is_use = reshape(is_use,h*w,1);
    input_points_2D = reshape(input_points,h*w,size(input_points,3));
    input_colors_2D = reshape(input_colors,h*w,size(input_colors,3));
    input_normals_2D = reshape(input_normals,h*w,size(input_normals,3));
    updated_points = reshape(proj_points,h*w,size(proj_points,3));
    updated_colors = reshape(proj_colors,h*w,size(proj_colors,3));
    updated_normals = reshape(proj_normals,h*w,size(proj_normals,3));
    updated_ccounts = reshape(proj_ccounts,h*w,1);
    updated_times = reshape(proj_times,h*w,1);
    alpha_1D = reshape(alpha,h*w,1);
    
    input_points_2D = input_points_2D(is_use,:);
    input_colors_2D = input_colors_2D(is_use,:);
    input_normals_2D = input_normals_2D(is_use,:);
    proj_points_2D = updated_points(is_use,:);
    proj_normals_2D = updated_normals(is_use,:);
    proj_ccounts_1D = updated_ccounts(is_use);
    alpha_1D = alpha_1D(is_use);
    
    updated_points(is_use,:) = ((proj_ccounts_1D.*proj_points_2D) + (alpha_1D.*input_points_2D)) ./ (proj_ccounts_1D+alpha_1D);    
    updated_normals(is_use,:) = ((proj_ccounts_1D.*proj_normals_2D) + (alpha_1D.*input_normals_2D))./ (proj_ccounts_1D+alpha_1D);
    updated_colors(is_use,:) = input_colors_2D;
    updated_ccounts(is_use,:) = proj_ccounts_1D+alpha_1D;
    updated_times(is_use,:) = t;
    
    %Reshaping back to their prior shapes
    updated_points = reshape(updated_points,[h,w,3]);
    updated_normals = reshape(updated_normals,[h,w,3]);
    updated_colors = reshape(updated_colors,[h,w,3]);
    updated_ccounts = reshape(updated_ccounts,[h,w]);
    updated_times = reshape(updated_times,[h,w]);
    
    
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end