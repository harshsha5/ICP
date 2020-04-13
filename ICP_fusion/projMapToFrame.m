function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====
    K = [fx 0 cx;0 fy cy; 0 0 1];   %Camera Intrinsic Matrix
    
    % Write your code here...
    camera_frame_point_cloud = pctransform(fusion_map.pointcloud, tform.invert);
    camera_coord = camera_frame_point_cloud.Location;
    proj_flag = true(size(camera_coord,1),1);
    indices_behind_camera = camera_coord(:,3) <= 0; %Only project the points in front of the camera (ie.z>0)
%     camera_coord(indices_behind_camera,:) = zeros(1,size(camera_coord,2));  %Only project the points in front of the camera (ie.z>0)
    camera_coord(indices_behind_camera,:) = 0;
    proj_flag(indices_behind_camera) = 0;
    
    image_coordinates = K*camera_coord';
    image_coordinates = image_coordinates./image_coordinates(size(image_coordinates,1),:);
    image_coordinates = image_coordinates(1:2,:);
    image_bounded_indices = image_coordinates(1,:)>0 & image_coordinates(1,:)<h & image_coordinates(2,:)>0 & image_coordinates(2,:)<w;
    image_coordinates(:,~image_bounded_indices) = 0;
    proj_flag(~image_bounded_indices) = 0;  %Validate this
    image_coordinates = ceil(image_coordinates);
    
    % Form a 3D matrix of the appropriate correspondences- pixel to
    % fusion_map mapping
    proj_points = zeros(h * w, 3);
    proj_colors = zeros(h * w, 3);
    proj_normals = zeros(h * w, 3);
    proj_ccounts = zeros(h * w,1);
    proj_times = zeros(h * w,1);
    
    non_zero_image_coordinates = image_coordinates(:,proj_flag');

    pixel_2_row_index = h*(non_zero_image_coordinates(1,:)') + non_zero_image_coordinates(2,:)'+1;
    proj_points(pixel_2_row_index,:) = fusion_map.pointcloud.Location(proj_flag,:);
    proj_colors(pixel_2_row_index,:) = fusion_map.pointcloud.Color(proj_flag,:);
    proj_normals(pixel_2_row_index,:) = fusion_map.normals(proj_flag,:);
    proj_ccounts(pixel_2_row_index) = fusion_map.ccounts(proj_flag,:);
    proj_times(pixel_2_row_index) = fusion_map.times(proj_flag,:);

    proj_points = reshape(proj_points,[h,w,3]);
    proj_colors = reshape(proj_colors,[h,w,3]);
    proj_normals = reshape(proj_normals,[h,w,3]);
    proj_ccounts = reshape(proj_ccounts,[h,w]);
    proj_times = reshape(proj_times,[h,w]);
        
    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
