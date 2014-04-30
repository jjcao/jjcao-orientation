%test visible  point
function testVisiblePoint(P)
a =[0.73,0.0,0.5;
    0.337,0.6,0.5;
    1.0,0.65,0.35;
    0.0,0.0,0.8125;
    0.0,1.0,1.0;
    1.0 0.0 0.0;
    0.5,0.5,0.5;
    127/255,1,212/255;
    0.0625,1.0,0.9375;
    1.0,1.0,0.0;
    0.5,0.0,0.0;
    1.0,0.375,0.0;
    0.75,0.0,0.0;
    0.5,0.0,0.5;
    ];
color=a;
figure('Name','visible vs not visible');movegui('northeast');set(gcf,'color','white'); axis off;axis equal;set(gcf,'Renderer','OpenGL');axis vis3d;view3d rot;hold on;
%画视点和可视点
file=fopen('../result/result_viewpoint.txt','rt');
num=fscanf(file,'%d\n',[1,1]);
for i=1:num
    vote_point=fscanf(file,'%d %d %d\n',[1,3]);
    visible_num=fscanf(file,'%d\n',[1,1]);
    for j=1:visible_num
        point(1,j)=fscanf(file,'%d ',[1,1]);
    end
    fscanf(file,'\n');
    scatter3(vote_point(1,1),vote_point(1,2),vote_point(1,3),600,'.','MarkerEdgeColor',color(i,:)); hold on;
    scatter3(P.pts(point(1,:),1),P.pts(point(1,:),2),P.pts(point(1,:),3),100,'.','MarkerEdgeColor',color(i,:)); hold on;
    %for k=1:visible_num
       %arrow3([vote_point(1,1) vote_point(1,2) vote_point(1,3)],[0 0 0],'b',0.5);hold on;
    %end
end
fclose(file);