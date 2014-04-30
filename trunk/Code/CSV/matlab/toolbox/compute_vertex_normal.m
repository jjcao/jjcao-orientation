function [normal,normalf] = compute_vertex_normal(vertex,face, bNormalized)

% compute_normal - compute the normal of a triangulation
%
%   [normal,normalf] = compute_vertex_normal(vertex,face);
%
%   normal(i,:) is the normal at vertex i.
%   normalf(j,:) is the normal at face j.
%
%   Add bNormalzied and take a ', 2009, JJCAO
%   Copyright (c) 2004 Gabriel Peyr?
normalf = crossp( vertex(face(:,2),:)-vertex(face(:,1),:), ...
                  vertex(face(:,3),:)-vertex(face(:,1),:) );
if nargin < 3
    bNormalized = 1;
end
if bNormalized
    d = sqrt( sum(normalf.^2,2) ); d(d<eps)=1;
    normalf = normalf ./ repmat( d, 1,3 );
end

% face normal to vertex normal
nface = size(face,1);
nvert = size(vertex,1);
normal = zeros(nvert,3);

for i=1:nface
    f = face(i,:);
    for j=1:3
        normal(f(j),:) = normal(f(j),:) + normalf(i,:);
    end
end
% normalize
if bNormalized
    d = sqrt( sum(normal.^2,2) ); d(d<eps)=1;
    normal = normal ./ repmat( d, 1,3 );
end

% enforce that the normal are outward
v = vertex - repmat(mean(vertex,2), 1,3);
s = sum( v.*normal, 1 );
if sum(s>0)<sum(s<0)
    % flip
    normal = -normal;
    normalf = -normalf;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function z = crossp(x,y)
% x and y are (m,3) dimensional
z = x;
z(:,1) = x(:,2).*y(:,3) - x(:,3).*y(:,2);
z(:,2) = x(:,3).*y(:,1) - x(:,1).*y(:,3);
z(:,3) = x(:,1).*y(:,2) - x(:,2).*y(:,1);
