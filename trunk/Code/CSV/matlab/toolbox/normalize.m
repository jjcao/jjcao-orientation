function vec = normalize(vec)
tmp = sqrt(vec(:,1).^2+vec(:,2).^2+vec(:,3).^2);
vec(:,1) =vec(:,1)./tmp;
vec(:,2) =vec(:,2)./tmp;
vec(:,3) =vec(:,3)./tmp;

% test normalize
% 1==sqrt(vec(:,1).^2+vec(:,2).^2+vec(:,3).^2)
