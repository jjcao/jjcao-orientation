% test thindist
%% setting
clear;clc;close all;
path('toolbox',path);
extension='.off';

step = 0.1;
num = 10;
x=[1:step:(1+num*step)]'; y = ones(num+1,1); z = zeros(num+1,1);
p1=[x,y,z]; p2=[x,y-0.5*step,z];
p=[p1;p2];
n = [0,1, 0];
n = repmat(n,size(p,1), 1);

d=zeros(size(p,1),1);
cp = 0.5*num;
for i=1:size(p,1)
    d(i) = thindist(p(cp,:), p(i,:), n(cp,:), n(i,:));
end
[B(:,1),B(:,2)]=sort(d);
P=[p(:,1:2) d];
%%
figure;movegui('northeast');set(gcf,'color','white'); axis off;axis equal;set(gcf,'Renderer','OpenGL');axis vis3d;view3d rot;hold on;
scatter3(P(:,1),P(:,2),P(:,3),60,P(:,3),'filled');  
for i=1:length(d)
    text(P(i,1),P(i,2),P(i,3),int2str(i)) ;
end
