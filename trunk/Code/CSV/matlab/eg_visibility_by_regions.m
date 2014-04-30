%
% create: 2010-11-02
% by: JJCAO
%
%% setting
clear;clc;close all;
path('toolbox',path);
% camposi = [1,0,0;-1,0,0;0,1,0;0,0,1;0,0,-1;];% 6 face - -Y axis
% camposi = [1,0,0;-1,0,0;0,1,0;0,-1,0;0,0,1;0,0,-1;];% 6 face
camposi = [1,0,0;-1,0,0;0,1,0;0,-1,0;0,0,1;0,0,-1;% 6 face
    1,-1,-1;  1,-1,1; 1,1,-1; 1,1,1; 
    -1,-1,-1;-1,-1,1;-1,1,-1; -1,1,1% 8 corner
    ];
% camposi = [0,1,0; 0,1,1; 0,0,1; 0,-1,1; 0,-1,0; 0,-1,-1; 0,0,-1; 0,1,-1;%x=0 plane
%            1,0,0; 1,0,1;        -1,0,1; -1,0,0; -1,0,-1;         1,0,-1;
%                   1,1,0; 1,0,0; 1,-1,0;         -1,-1,0; -1,0,0; -1,1,0;
%     ];%20
contrno = 1;
param = 2;% large value for large view, 5 is too large

fprintf('eg_visibility_by_regions: %d view \n', size(camposi,1));
%% input
sk_filename='../data/vase-lion_98k_contract_nn(7)_WL(28.504436)_WH(2.000000).mat';
load(sk_filename,'P');
P.contrno = contrno;
p= [P.pts; P.cpts{contrno}];

% figure('Name','Original point cloud and its contraction');movegui('northwest');set(gcf,'color','white');
% scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),10,'.','MarkerEdgeColor',GS.PC_COLOR); axis off;axis equal; hold on;
% cpts=P.cpts{contrno};
% scatter3(cpts(:,1),cpts(:,2),cpts(:,3),30,'.r');
% view3d rot;

%% visibility
% figure;movegui('northeast');set(gcf,'color','white');axis off;axis equal; hold on;view3d rot;
% scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),30,'.','MarkerEdgeColor',GS.PC_COLOR); 
tic
P.tag = zeros(P.npts, 2);
for i=1:size(camposi,1);
    visiblePtInds=HPR(p,camposi(i,:),param);
    vp = visiblePtInds(visiblePtInds>P.npts)-P.npts;%visible points of contracted points
    vptp = visiblePtInds(visiblePtInds<(P.npts+1));%visible points of original points
    
    for j = vp'
        if P.patch(j) == -1
            continue;
        end        
        tmp = find(P.patch==P.patch(j));
        P.tag(tmp,1) = P.tag(tmp,1) + 1;
    end
    for j = vptp'
        if P.patch(j) == -1
            continue;
        end        
        tmp = find(P.patch==P.patch(j));
        P.tag(tmp,2) = P.tag(tmp,2) + 1;
    end    
    
%     cpts=p(vp+P.npts,:);
%     scatter3(cpts(:,1),cpts(:,2),cpts(:,3),60,'.r');
end
% camorbit(0,0,'camera'); axis vis3d; view(90,0);

%% adjust normal1 estimated by tangent plane
% Do not change initial normal if we do not see the point or contracted
% point
P.normal3=P.normal2;
tag = sign( P.tag(:,2)-P.tag(:,1) );

for i=1:size(tag)
    if tag == 0 % not saw
        P.normal3(i,:) = P.normal1(i,:);   
    else
        P.normal3(i,:) = P.normal3(i,:)*tag(i);
        if dot(P.normal1(i,:),P.normal3(i,:)) < 0.001
            P.normal3(i,:) = -P.normal1(i,:);
        else
            P.normal3(i,:) = P.normal1(i,:);        
        end
    end
end
toc
%% save results
write_mesh('../result/result.off', P.pts,[],P.normal3);
write_mesh('../result/result_c.off',  P.cpts{contrno},[],P.normal1);

save(sk_filename,'P');