%
% create: 2010-11-02
% by: JJCAO
%
%% setting
clear;clc;close all;
path('toolbox',path);
% camposi = [1,0,0;];
camposi = [1,0,0;-1,0,0;0,1,0;0,-1,0;0,0,1;0,0,-1];
contrno = 1;
param = 2;% large value for large view
%%
sk_filename='../data/Ness_v13790_contract_nn(30)_WL(18.294663)_WH(2.000000).mat';
load(sk_filename,'P');
p= [P.pts; P.cpts{contrno}];

% figure('Name','Original point cloud and its contraction');movegui('northwest');set(gcf,'color','white');
% scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),10,'.','MarkerEdgeColor',GS.PC_COLOR); axis off;axis equal; hold on;
% cpts=P.cpts{contrno};
% scatter3(cpts(:,1),cpts(:,2),cpts(:,3),30,'.r');
% view3d rot;

%%
figure;movegui('northeast');set(gcf,'color','white');axis off;axis equal; hold on;view3d rot;
% scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),30,'.','MarkerEdgeColor',GS.PC_COLOR); 

P.normal2 = P.pts-P.cpts{contrno};
tmp = sqrt(P.normal2(:,1).^2+P.normal2(:,2).^2+P.normal2(:,3).^2);
P.normal2(:,1) =P.normal2(:,1)./tmp;
P.normal2(:,2) =P.normal2(:,2)./tmp;
P.normal2(:,3) =P.normal2(:,3)./tmp;

P.tag = zeros(P.npts, 1);
vp = cell(size(camposi,1));%visible points of contracted points from diff views
for i=1:size(camposi,1);
    visiblePtInds=HPR(p,camposi(i,:),param);
    vp{i} = visiblePtInds(visiblePtInds>P.npts);
    vptp = setdiff(visiblePtInds, vp{i});
    
    tmp = setdiff(vptp, vp{i}-P.npts);%remove any index of which both contracted and init point can be seen.
    vp{i} = setdiff(vp{i},vptp+P.npts);
    vptp = tmp; clear tmp;
    
    P.tag(vptp) = P.tag(vptp) + 1;
    P.tag(vp{i}-P.npts) = P.tag(vp{i}-P.npts) + 1;
    
    scatter3(P.pts(vptp,1),P.pts(vptp,2),P.pts(vptp,3),30,'.','MarkerEdgeColor',GS.PC_COLOR); 
%     cpts=p(vp{i},:);
%     scatter3(cpts(:,1),cpts(:,2),cpts(:,3),60,'.r');
end
camorbit(0,0,'camera'); axis vis3d; view(90,0);

% %% adjust normal1 estimated by Laplacian contraction
% newvp = [];
% for i=1:size(vp)
%     newvp = union(newvp,vp{i});
% end
% tag = ones(P.npts,1);
% tag(newvp-P.npts) = -1;
% for i=1:size(tag)
%     P.normal2(i,:) = P.normal2(i,:)*tag(i);
% end

%% adjust normal1 estimated by local neighbor
% Do not change initial normal if we do not see the point or contracted
% point
P.normal1 = -P.normal1; % assume P.normal1 is right
newvp = [];
for i=1:size(vp)
    newvp = union(newvp,vp{i});
end
tag = ones(P.npts,1);
tag(newvp-P.npts) = -1;
for i=1:size(tag)
    if P.tag(i) == 0 % not saw
        P.normal2(i,:) = P.normal1(i,:);   
    else
        P.normal2(i,:) = P.normal2(i,:)*tag(i);
        if dot(P.normal1(i,:),P.normal2(i,:)) < 0.001
            P.normal2(i,:) = -P.normal1(i,:);
        else
            P.normal2(i,:) = P.normal1(i,:);        
        end
    end
end
%% save results
write_mesh('../result/result.off', P.pts,[],P.normal2);
write_mesh('../result/result_c.off',  P.cpts{contrno},[],P.normal1);

save(sk_filename,'P');