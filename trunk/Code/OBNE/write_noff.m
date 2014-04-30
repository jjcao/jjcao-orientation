function []=write_noff(filename,verts,normals)

%   write_noff(filename, vertex, normals);
%
%   vertex must be of size [n,3]
%   normals must be of size [p,3]
%
fid=fopen(filename,'wt');
if(fid==-1)
    error('can''t open the file');
    return;
end
%header
fprintf(fid,'NOFF\n');
fprintf(fid, '%d 0\n', size(verts,1));
%write the points & normals
verts=[verts normals];
fprintf(fid,'%f %f %f %f %f %f\n',verts');
fclose(fid);