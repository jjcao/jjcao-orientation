function dij = thindist(pi,pj,ni,nj)
% Formula 1) of Consolidation of Unorganized Point Clouds for Surface Reconstruction
% 2010/11/23 jjcao
vij = pj-pi;
if dot(vij,vij) == 0
    dij = 1;
    return;
end

xi = [pi + ni; pi - ni]; xi=[xi;xi];
xj = [pj + nj; pj - nj]; xj=[xj;xj(2,:);xj(1,:)];
m = 0.5*( xi+xj );
v1 = m-repmat(pi,size(m,1), 1);
nvij = normalize(vij);
l = dot(v1 ,repmat(nvij,size(m,1),1), 2);
o = repmat(l,1,3).*repmat(nvij,size(m,1),1) + repmat(pi,size(m,1), 1);
tmp = dot(m-o, m-o,2).^0.5;
dij = 1 - dot(ni,nj)* max(tmp)/(1+sqrt( dot(vij,vij) ));
% return;

% function vec = normalize2(vec)
% tmp = sqrt(vec(:,1).^2+vec(:,2).^2);
% vec(:,1) =vec(:,1)./tmp;
% vec(:,2) =vec(:,2)./tmp;
