function [P default_filename]= contraction(P)
% contract a point cloud by Laplacian, for orinating
% create: 2010-8-23
% by: JJCAO
%
SHOW_RESULT=1;
SAVE_RESULT=1;
%% Step 1: build local 1-ring and normal
if  isempty(P.normal1)
    P.k_knn = GS.compute_k_knn(P.npts);
    kdtree = kdtree_build(P.pts);% kdtree,用来找k近邻
    index = zeros(P.npts, P.k_knn);
    P.normal1 = zeros(P.npts, 3);
    for i = 1:P.npts
        index(i,:)  = kdtree_k_nearest_neighbors(kdtree,P.pts(i,:),P.k_knn)';
        P.normal1(i,:) = compute_lsp_normal(P.pts(index(i,:),:));
    end
    kdtree_delete(kdtree);
    clear index;
else
    P.normal1 = -P.normal1; % normal by read always right, so we flip them.
end

%--------------------
tic
P.k_knn = 10;% 9 or 7

P.rings = cell(P.npts,1);
kdtree = kdtree_build(P.pts);% kdtree,用来找k近邻
index = zeros(P.npts, P.k_knn);
for i = 1:P.npts
    index(i,:)  = kdtree_k_nearest_neighbors(kdtree,P.pts(i,:),P.k_knn)';
    %P.rings{i}=index(i,2:end);
end
fprintf('KNN: '); toc

%--------------------
tic
tmp= P.k_knn - min(4,P.k_knn/2);
P.index = zeros(P.npts,tmp);
for i = 1:P.npts
    neighs = index(i,2:end);
    len = length(neighs);
    d=zeros(len,1);
    for j=1:len
        d(j) = thindist(P.pts(i,:), P.pts(neighs(j),:), P.normal1(i,:), P.normal1(neighs(j),:));
    end
    B=zeros(len,2);
    [B(:,1),B(:,2)]=sort(d);
    P.rings{i}=neighs(B(1:tmp-1,2));
%     P.index(i,:)=[i, P.rings{i}];
end
%-------------------
fprintf('compute neighbor:'); toc
tic
% P.rings = compute_point_point_ring(P.pts, P.k_knn, P.index);
fprintf('compute local 1-ring:');toc

% P.index=P.index(:,2:end);
kdtree_delete(kdtree);
clear index;

%% Step 2: Contract point cloud by Laplacian
tic
options.WC = 2;
[P.cpts, initWL, WC, P.L0] = contraction_by_mesh_laplacian(P, options);
fprintf('Contraction:'); toc

%% show results
if ~SHOW_RESULT
    return;
end

figure('Name','Original point cloud and its contraction');movegui('northeast');set(gcf,'color','white')
if isfield(P,'faces') && ~isempty(P.faces)
    options.face_color = [0 1 0];
    h = plot_mesh(P.pts, P.faces, options); hold on;colorbar('off');alpha(0.5);
    set(h, 'edgecolor', 'none'); % cancel display of edge.
else
    scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),30,'.','MarkerEdgeColor',GS.PC_COLOR);  hold on;
end

cpts=P.cpts{end};
scatter3(cpts(:,1),cpts(:,2),cpts(:,3),30,'.r'); axis off;axis equal;set(gcf,'Renderer','OpenGL');
camorbit(0,0,'camera'); axis vis3d; view(0,90);view3d rot;

%% save results
default_filename = sprintf('%s_contract_nn(%d)_WL(%f)_WH(%f).mat',...
    P.filename(1:end-4), P.k_knn, initWL, WC);
if SAVE_RESULT
    save(default_filename,'P');
end