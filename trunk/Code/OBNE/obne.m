%orientation-benefit normal estimation algorithm(OBNE)
clear
clc
%读取点云数据
database = 'closeby'; % sharpFeature,closeby,noise,nonuniform,topology
[imdir, spdir] = gene_dir(database);
imnames=dir([imdir '*' 'off']);
for i=1:length(imnames)
    filename=imnames(i).name;
    file=filename(1:end-4)
    format=imnames(i).name(end-3:end);
    [point faces normal]=read_off([imdir filename]);
%     [point normal]=read_noff([imdir filename]);
    %PCA估计法向量
    tic
    [vecm,index]=compute_normal_pca(point,6);
    toc
    %用OBNE估计法向量
    tic
    for i1=1:size(point,1)
        st=zeros(3);
        %考虑距离权重
        ii=1;
        for j11=2:(size(index,2)-1) 
            for j22=(j11+1):size(index,2)
                rr(1,ii)=norm(point(index(i1,j11),:)-point(index(i1,j22),:));
                ii=ii+1;
            end
        end     
        r=max(rr(1,:))/2;

        for j1=2:(size(index,2)-1)
            for j2=(j1+1):size(index,2)
            pk(:,1)=point(index(i1,j1),:)-point(index(i1,j2),:);
            if pk==[0;0;0]
                continue;
            end
            %加入Hoppe权
            mr1=point(index(i1,j1),:)+vecm(index(i1,j1),:);
            mr2=point(index(i1,j1),:)-vecm(index(i1,j1),:);
            ms1=point(index(i1,j2),:)+vecm(index(i1,j2),:);
            ms2=point(index(i1,j2),:)-vecm(index(i1,j2),:);
            mrs11=(mr1+ms1)/2;
            mrs12=(mr1+ms2)/2;
            mrs21=(mr2+ms1)/2;
            mrs22=(mr2+ms2)/2;
            aa11=point(index(i1,j1),:)-mrs11;
            bb11=point(index(i1,j2),:)-mrs11;
            aa12=point(index(i1,j1),:)-mrs12;
            bb12=point(index(i1,j2),:)-mrs12;
            aa21=point(index(i1,j1),:)-mrs21;
            bb21=point(index(i1,j2),:)-mrs21;
            aa22=point(index(i1,j1),:)-mrs22;
            bb22=point(index(i1,j2),:)-mrs22;
            cc11=norm(cross(aa11,bb11))/norm(point(index(i1,j1),:)-point(index(i1,j2),:));
            cc12=norm(cross(aa12,bb12))/norm(point(index(i1,j1),:)-point(index(i1,j2),:));
            cc21=norm(cross(aa21,bb21))/norm(point(index(i1,j1),:)-point(index(i1,j2),:));
            cc22=norm(cross(aa22,bb22))/norm(point(index(i1,j1),:)-point(index(i1,j2),:));
            djk=max([cc11,cc12,cc21,cc22]);
            ddjk=1-abs(dot(vecm(index(i1,j1),:),vecm(index(i1,j2),:)))*djk/(1+norm(point(index(i1,j1))-point(index(i1,j2))));
            %考虑距离权重
            wjk=exp(-norm(point(index(i1,j1),:)-point(index(i1,j2),:))^2/r^2);
            end
        end
            [v,d]=eig(st);
            dd=min(diag(d));
            switch dd
                case d(1,1)
                     vec(i1,:)=v(:,1)';
                case d(2,2)
                     vec(i1,:)=v(:,2)';
                case d(3,3)
                      vec(i1,:)=v(:,3)';
            end 
             t=sqrt(vec(i1,1)^2+vec(i1,2)^2+vec(i1,3)^2);
             vec(i1,:)=vec(i1,:)/t;

           point_normal(i1,:)=vec(i1,:);
    end
    toc
    %用我们的方法计算法向。不做定向。定向的工作用NormalOrientation程序做。
    write_noff(['dataOBNE/' database '/' file '_obne_unorientation' format],point,point_normal);
end