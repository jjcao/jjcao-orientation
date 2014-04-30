clear;clc;close all;
path(path,'toolbox') ;
%%
filename='../data/bunny.off';
[P.pts,tmp, P.normal] = read_mesh(filename);

bbox = [min(P.pts(:,1)), min(P.pts(:,2)), min(P.pts(:,3)), max(P.pts(:,1)), max(P.pts(:,2)), max(P.pts(:,3))];
rs = bbox(4:6)-bbox(1:3);
radius = sqrt(dot(rs,rs))*0.5;

type = 'random';
% type = 'gaussian';
 p3 = 0.5*0.01;
%p3 = radius/1280*1.35;%radius/1280 too much of noise
% type = 'salt & pepper';
% p3 = 0.01;%0.01 is too bad

opts = pcd_noise(P.pts, type, p3);
figure;set(gcf,'color','white');movegui('northeast');set(gcf,'Renderer','OpenGL');hold on; 
scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),30,'.','MarkerEdgeColor','red'); 
scatter3(opts(:,1),opts(:,2),opts(:,3),30,'.','MarkerEdgeColor',GS.PC_COLOR); 
axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(00,90);view3d rot;

ofilename = sprintf('%s_%s_%f.off',filename(1:(end-4)), type, p3);
write_mesh(ofilename, opts,[],[]);
% ofilename = sprintf('%s_%s_%f.xyz',filename(1:(end-4)), type, p3);
% write_mesh(ofilename, opts,[],M.normals);