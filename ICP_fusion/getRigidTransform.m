function [tform valid_pair_num error] = getRigidTransform(new_pointcloud, ref_pointcloud, ref_normals)
    
    %==== Initialize parameters ====
    iter_num = 6;
    d_th = 0.05;
    m = size(new_pointcloud.Location, 1);
    n = size(new_pointcloud.Location, 2);
    tform = affine3d(eye(4));
    
    %==== Main iteration loop ====
    for iter = 1:iter_num
        
        %==== Set variables ====
        new_pts = new_pointcloud.Location;
        ref_pts = ref_pointcloud.Location;
        
        %==== For each reference point, find the closest new point within a local patch of size 3-by-3 ====        
        %==== (Notice: assoc_pts[] has the same size and format as new_pts[] and ref_pts[]) ====
        %==== (Notice: assoc_pts[i, j, :] = [0 0 0] iff no point in new_pts[] matches to ref_pts[i, j, :]) ====
        assoc_pts = findLocalClosest(new_pts, ref_pts, m, n, d_th);
        
        %==== Set the sizes of matrix A[] and vertor b[] of normal equation: A'*A*x = A'*b ====
%         A = zeros(m*n, 6);
%         b = zeros(m*n, 1);
        
        %==== declare the number of point pairs that are used in this iteration ==== 
%         valid_pair_num = 0;
        assoc_pts_2D = reshape(assoc_pts,[m*n, 3]);
        zero_counts = sum(~assoc_pts_2D,2);
        valid_pair_num = size(assoc_pts_2D,1) - sum(zero_counts()==3);  %Get number of valid  associated_pts
        non_zero_rows = zero_counts()~=3;
        valid_assoc_pts_2D = assoc_pts_2D(non_zero_rows , :);   %Get valid  associated_pts
        A = zeros(size(valid_assoc_pts_2D,1), 6);
        
        %==== TODO: Assign values to A[] and b[] ====
        %==== (Notice: the format of the desired 6-vector is: xi = [beta gamma alpha t_x t_y t_z]') ====
        ref_pts_2D = reshape(ref_pts,[m*n, 3]);
        valid_ref_pts_2D = ref_pts_2D(non_zero_rows , :); 
        
        ref_normals_2D =  reshape(ref_normals,[m*n, 3]);
        valid_ref_normals_2D = ref_normals_2D(non_zero_rows , :); 
        
        b = sum((valid_ref_pts_2D - valid_assoc_pts_2D).*valid_ref_normals_2D,2);
        
        % Write your code here...
%         b = sum((ref_pts - assoc_pts).*ref_normals,3); %See if we need to remove the rows which came with points of no association
%         b = reshape(b,[m*n, 1]); %First all column points are taken then the next column is appended at the bottom
        %==== TODO: Solve for the 6-vector xi[] of rigid body transformation ====

        % Write your code here...
        A(:,4:6) = valid_ref_normals_2D;
        A(:,1) = valid_ref_normals_2D(:,3).* valid_assoc_pts_2D(:,2) - valid_ref_normals_2D(:,2).* valid_assoc_pts_2D(:,3);
        A(:,2) = valid_ref_normals_2D(:,1).* valid_assoc_pts_2D(:,3) - valid_ref_normals_2D(:,3).* valid_assoc_pts_2D(:,1);
        A(:,3) = valid_ref_normals_2D(:,2).* valid_assoc_pts_2D(:,1) - valid_ref_normals_2D(:,1).* valid_assoc_pts_2D(:,2);
        
        %pseudoinverse method to solve
        A_pseudo = pinv(A);
        xi = A_pseudo*b;
       
        %Chol2 to solve
%         As = sparse(A);
%         [xi, ~] = solve_chol1(As, b);
% %         xi = xi';

        
        %==== Coerce xi[] back into SE(3) ====
        %==== (Notice: tmp_tform[] is defined in the format of right-multiplication) ====
        R = toSkewSym(xi(1:3)) + eye(3);
        [U,S,V] = svd(R);
        R = U*V';
        T = [R [0 ; 0 ; 0] ; [xi(4:6)' 1]];
        tmp_tform = affine3d(T);
        
        %==== Updates the transformation and the pointcloud for the next iteration ====
        %==== (uses affine3d() and pctransform() functions) ====
        %==== (note the format of tform[] and the affine3d() function) ====
        tform = affine3d(tmp_tform.T*tform.T);
        if iter ~= iter_num
            new_pointcloud = pctransform(new_pointcloud, tmp_tform);
        end
        
    end
    
    %==== Find RMS error of point-plane registration ====
    error = sqrt(sum((A*xi - b).^2)/valid_pair_num);
end
        
