%
% create: 2010-11-19
% by: JJCAO
%
%% setting
clear;clc;close all;
path('toolbox',path);
disp('eg_patch')

sk_filename='../data/vase-lion_98k_contract_nn(7)_WL(28.504436)_WH(2.000000).mat';
load(sk_filename,'P');

%% normailze contraction direction
contrno = 1;
P.normal2 = P.pts-P.cpts{contrno};
P.normal2 = normalize(P.normal2);

patch = zeros(P.npts,1);
for i=1:P.npts  
    if abs( dot(P.normal2(i,:),P.normal1(i,:)) ) < 0.3 %85 degree
        patch(i) = -1;
    end
end

%% patch by contraction direction
tic
% kdtree = kdtree_build( P.pts );
for i=1:P.npts  
    if patch(i)~=0, continue, end;
    
    patch(i) = max(patch)+1;
%     neighs = kdtree_k_nearest_neighbors( kdtree, P.pts(i,:), 5); neighs=neighs(2:end)';
    neighs = P.rings{i};
    for j = neighs
        if patch(j) < 0, continue, end;
        
        if dot(P.normal2(i,:),P.normal2(j,:)) > 0.9
            if patch(j) == 0 || patch(j)==patch(i)
                patch(j) = patch(i);
            else %更新所有的patch(j)
                patch(patch==patch(i))=patch(j);
            end
        end        
    end
end

% kdtree_delete( kdtree );
toc
%%
figure;movegui('northeast');set(gcf,'color','white'); axis off;axis equal;set(gcf,'Renderer','OpenGL');axis vis3d;view3d rot;hold on;
scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),60,patch+1,'filled');  
cmap = colormap(lines);

tmp = (patch==-1);
figure;movegui('southeast');set(gcf,'color','white'); axis off;axis equal;set(gcf,'Renderer','OpenGL');axis vis3d;view3d rot;hold on;
scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),10,'.g');  
scatter3(P.pts(tmp,1),P.pts(tmp,2),P.pts(tmp,3),60,'.r');  

P.patch = patch;
save(sk_filename,'P');
%%
tmp = (patch+1)/max(patch)*62 + 1;
color = zeros(P.npts,3);

for i = 1:P.npts
    x1 = floor(tmp(i));    x2 = ceil(tmp(i));
    y1 = cmap(x1,:);       y2 = cmap(x2,:);
    if x1==x2
        color(i,:) = y1;
    else
        u = (tmp(i)-x1)/(x2-x1);
        color(i,:) = y1*(1-u)+y2*u;
    end
end
write_mesh('../result/color.off', P.pts,[], P.normal1, color);