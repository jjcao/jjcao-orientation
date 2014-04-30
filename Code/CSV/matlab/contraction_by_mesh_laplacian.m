function [cptcell, initWL, WC, L] = contraction_by_mesh_laplacian(P, options)
% point cloud contraction_by_mesh_laplacian
% refer to Skeleton Extraction by Mesh Contraction 08
% mesh could be simplified by MeshLab/Filter/Clustering Decimation, if it
% is too larged.
% 
% inputs:
%   P.pts
%   P.faces
%   P.npts
%   P.k_knn: k of knn
%   P.rings:
%   options.WL = 3;% initial contraction weight
%   options.WC = 1;% initial attraction weight
%
% outputs:
%   cptcell: contracted vertices
%
% @author: jjcao
% @create-data:     2011-11-21
% profile on;

%##########################################################################
%% setting
%##########################################################################
Laplace_type = 'combinatorial';%conformal%combinatorial%spring%mvc

initWL = getoptions(options, 'WL', GS.compute_init_laplacian_constraint_weight(P,Laplace_type)); 
% set initial attraction weights according to different type of discrete
% Laplacian
if strcmp(Laplace_type,'mvc')
    WC = getoptions(options, 'WC', 1)*10;
elseif strcmp(Laplace_type,'conformal')
    WC = getoptions(options, 'WC', 1);
else
    WC = getoptions(options, 'WC', 1);
end
WH = ones(P.npts, 1)*WC; % ≥ı º‘º ¯»®
WL = initWL;%*sl;

sprintf(['1) k of knn: %d\n 3) Init Contract weight: %f\n 4) Init handle weight: %f\n'], P.k_knn, initWL, WC)

cptcell = cell(0);
%% left side of the equation
tic
L = -compute_point_laplacian(P.pts,Laplace_type,P.rings, options);%conformal;%spring
disp(sprintf('compute L:'));
toc

tic
A = [L.*WL;sparse(1:P.npts,1:P.npts, WH)];
% right side of the equation
b = [zeros(P.npts,3);sparse(1:P.npts,1:P.npts, WH)*P.pts];
cpts=(A'*A)\(A'*b); 
cptcell{end+1}=cpts;
% newVertices = A\b; % this is slow than above line
disp(sprintf('solve equation:'));
toc


