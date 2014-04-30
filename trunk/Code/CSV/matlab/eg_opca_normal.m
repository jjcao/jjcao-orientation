%orientation aware pca normal estimation
clear;clc;close all;
path('toolbox',path);

%% from point set off to point set obj
extension='.off';
filename = 'E:\jjcao_paper\PCD_Orientation\result\mechpart_faceNormal_ContV_14view_pts_thinL10';% which file we should run on
P.filename = [filename extension];
[P.pts,P.faces, P.normal2] = read_mesh(P.filename);
P.npts = size(P.pts, 1);


P.k_knn = 10;
kdtree = kdtree_build(P.pts);% kdtree,ÓÃÀ´ÕÒk½üÁÚ
index = zeros(P.npts, P.k_knn);
P.normal1 = zeros(P.npts, 3);
for i = 1:P.npts
    index  = kdtree_k_nearest_neighbors(kdtree,P.pts(i,:),P.k_knn)';
    tmp = [];
    for j = index
        if dot(P.normal2(i,:),P.normal2(j,:)) > 0.071
            tmp(end+1)=j;
        end
    end
    P.normal1(i,:) = compute_lsp_normal(P.pts(tmp,:));
end
kdtree_delete(kdtree);


write_mesh([filename '.off'], P.pts,[], P.normal1);