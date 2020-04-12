function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    fusion_map_pointcloud_location = fusion_map.pointcloud.Location(~proj_flag,:);
    fusion_map_pointcloud_color = fusion_map.pointcloud.Color(~proj_flag,:);
    fusion_map_normals = fusion_map.normals(~proj_flag,:);
    fusion_map_ccounts = fusion_map.ccounts(~proj_flag,:);
    fusion_map_times = fusion_map.times(~proj_flag,:);
      
    %Reshaping new information to combine easily
    updated_points_2D = reshape(updated_map.points, [h * w, 3]);
    updated_colors_2D = reshape(updated_map.colors, [h * w, 3]);
    updated_normals_2D = reshape(updated_map.normals, [h * w,3]);
    updated_ccounts_2D = reshape(updated_map.ccounts, [h * w, 1]);
    updated_times_2D = reshape(updated_map.times, [h * w, 1]);
    
    %Concatenate new updated points and old points from the prior fusion
    %map
    map_points = cat(1, fusion_map_pointcloud_location, updated_points_2D);
    map_colors = cat(1, fusion_map_pointcloud_color, updated_colors_2D);
    map_normals = cat(1, fusion_map_normals, updated_normals_2D);
    map_ccounts = cat(1, fusion_map_ccounts, updated_ccounts_2D);
    map_times = cat(1, fusion_map_times, updated_times_2D);
        
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   