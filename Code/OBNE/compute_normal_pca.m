function [vec,index]=dhuang(point,k)
%计算K阶邻
atris=nn_prepare(point);
[index,distance]=nn_search(point,atris,point,k);
%用PCA计算点法向量
for i1=1:size(point,1)
    st=zeros(3);
    for j1=2:size(index,2)
        pk(:,1)=point(i1,:)-point(index(i1,j1),:);
        st=st+pk*pk';
    end
        [v,d]=eig(st);
        dd=min([d(1,1) d(2,2) d(3,3)]);
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
end


