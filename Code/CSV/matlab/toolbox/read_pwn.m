function [verts, normals] = read_pwn(filename)

% read_pwn - read data from pwn file.
%
%   [verts, faces, normals] = read_pwn(filename);
%
%   'verts' is a 'nb.vert x 3' array specifying the position of the vertices.
%   'faces' is a 'nb.face x 3' array specifying the connectivity of the mesh.
%   'normals' is a 'nb.normal * 3' array specifying the normal of each vertex.
%   Copyright (c) 2010 JJCAO

fid = fopen(filename,'r');
if( fid==-1 )
    warning('Can''t open the file.');
    verts = [];normals=[];
    return;
end

str = fgetl(fid);
nverts = sscanf(str,'%d');

[A,cnt] = fscanf(fid,'%f %f %f', 6*nverts);
if cnt~=6*nverts
    warning('Problem in reading vertices.');
end
A = reshape(A, 3, cnt/3);
A = A';
verts = A(1:nverts,:);
normals = A(nverts+1:end,:);

fclose(fid);
return;