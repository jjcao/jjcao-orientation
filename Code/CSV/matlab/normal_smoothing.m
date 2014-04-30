function P = normal_smoothing(P, sk_filename)
%
% Smoothing orientation by Laplacian smoothing
% create: 2010-11-08
% by: JJCAO
%
%% setting
IS_DEBUG = 1;
SHOW_RESULT = 1;
SAVE_RESULT=1;
if ~exist('sk_filename','var')
%     clear;clc;close all;
    path('toolbox',path);
    sk_filename='../data/Moai_contract_nn(10)_WL(7.212904)_WH(2.000000).mat';
    load(sk_filename,'P');
end
fprintf('eg_normal_smoothing \n');

%%--------------------------------------------------------
tic
tag = abs(P.tag(:,2)-P.tag(:,1));
WH = tag.^(2);
A = [P.L0;sparse(1:P.npts,1:P.npts, WH)];

b = [zeros(P.npts,3);sparse(1:P.npts,1:P.npts, WH)*P.normal3];
nnormal=(A'*A)\(A'*b); 

fprintf('init smoothing: ');
toc

tic
nnormal = normalize(nnormal);
% adjust initial normal by smoothed orientation
for i=1:P.npts  
    if dot(P.normal1(i,:),nnormal(i,:)) < 0
        P.normal3(i,:) = -P.normal1(i,:);
    else
        P.normal3(i,:) = P.normal1(i,:);        
    end
end
fprintf('init flipping: ');
toc
%%--------------------------------------------------------
% update patch, belive majority
tic
patch = zeros(P.npts,1);
for i=1:P.npts  
    if patch(i)~=0, continue, end;
    
    patch(i) = max(patch)+1;
    neighs = P.rings{i};
%     neighs = P.index(i,1:2);
    for j = neighs        
        if dot(P.normal3(i,:),P.normal3(j,:)) > 0
            if patch(j) == 0 || patch(j)==patch(i)
                patch(j) = patch(i);
            else % update all patch(j)
                patch(patch==patch(i))=patch(j);
            end
        end        
    end
end
toc
tic
% update tag, belive majority, method new
tmp1 = unique(patch);
tmp2 = tmp1;
len = length(tmp1);
for i = 1:len
    tmp2(i) = sum( patch==tmp1(i));
end
% tmp = mean(tmp2); 
tmp = max(tmp2)-1;%是否再多相信两个，还没想好。
for i = 1:len
    if tmp2(i) > tmp
%         tag(patch==tmp1(i)) = max( max( abs(tag(patch==tmp1(i))) ),P.npts); %Matrix is close to singular or badly scaled.
%         tag(patch==tmp1(i)) = max( max(tag), 999);
    tag(patch==tmp1(i)) = max(tag);
    else
        tag(patch==tmp1(i)) = 0;
    end
end
fprintf('cluster by orientation: ');
toc

tic
WH = tag.^(2);
A = [P.L0;sparse(1:P.npts,1:P.npts, WH)];

b = [zeros(P.npts,3);sparse(1:P.npts,1:P.npts, WH)*P.normal3];
nnormal=(A'*A)\(A'*b); 
fprintf('final smoothing: ');
toc

tic
nnormal = normalize(nnormal);
% adjust initial normal by smoothed orientation
for i=1:P.npts  
    % how to determin this parameter, both angle relative the P.normal1 and size of nnormal 
    % reflect whether should we flip the P.normal1!!!!!!!!!!!!!!!!!!!!!
    if dot(P.normal1(i,:),nnormal(i,:)) < 0
        P.normal3(i,:) = -P.normal1(i,:);
    else
        P.normal3(i,:) = P.normal1(i,:);        
    end
end
%%
fprintf('final slipping: ');
toc
write_mesh('../result/result.off', P.pts,[],P.normal3);
% write_mesh('../result/result_c.off',  P.cpts{P.contrno},[],P.normal1);
write_mesh('../result/result.pwn', P.pts,[], P.normal3);

%%--------------------------------------------------------
figure;movegui('northeast');set(gcf,'color','white'); axis off;axis equal;set(gcf,'Renderer','OpenGL');axis vis3d;view3d rot;hold on;
scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),60,patch+1,'filled');  
cmap = colormap(lines);

%%
tmp = (patch)/max(patch)*63 + 1;
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
write_mesh('../result/color.off', P.pts,[], P.normal3, color);

