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
    proj_points = zeros(h , w, 3);
    proj_colors = zeros(h , w, 3);
    proj_normals = zeros(h , w, 3);
    proj_ccounts = zeros(h , w);
    proj_times = zeros(h , w);
    
    for i = 1:size(image_coordinates,2)
        if(image_coordinates(:,i) == zeros(2,1))
            continue
        end
        proj_points(image_coordinates(1,i),image_coordinates(2,i),:) = fusion_map.pointcloud.Location(i,:);
        proj_colors(image_coordinates(1,i),image_coordinates(2,i),:) = fusion_map.pointcloud.Color(i,:);
        proj_normals(image_coordinates(1,i),image_coordinates(2,i),:) = fusion_map.normals(i,:);
        proj_ccounts(image_coordinates(1,i),image_coordinates(2,i)) = fusion_map.ccounts(i,:);
        proj_times(image_coordinates(1,i),image_coordinates(2,i)) = fusion_map.times(i,:);
    end
        
    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
