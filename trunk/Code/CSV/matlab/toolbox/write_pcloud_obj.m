function [] = write_pcloud_obj(filename, verts, normals, type)

% write_point_obj - write a point cloud to an OBJ file
%
%   write_point_obj(filename, verts, normals, type)
%
%   verts must be of size [n,3]
%
%   Copyright (c) 2009 JJCAO
% filename = '../data/1338_Galaad_clean.obj';
% [M.verts,M.faces] = read_mesh(filename);
% write_point_obj('test1.obj',M.verts, M.normals, 'studio');

fid = fopen(filename,'wt');
if( fid==-1 )
    error('Can''t open the file.');
    return;
end

switch lower(type)
    case 'andrea'
        write_pcloud_obj_andrea(fid, verts, normals);
    case 'studio'
        write_pcloud_obj_studio(fid, verts, normals);
end
fclose(fid);

function [] = write_pcloud_obj_andrea(fid, verts, normals)
nverts = size(verts, 1);
% vertex position
for i = 1:nverts
    fprintf(fid, 'v %f %f %f\n', verts(i,:));
end
% normals
if ~isempty(normals)
    for i = 1:nverts
        fprintf(fid, 'vn %f %f %f\n', normals(i,:));
    end
end

function [] = write_pcloud_obj_studio(fid, verts, normals)
nverts = size(verts, 1);

fprintf(fid, '# Studio\n');
fprintf(fid, 'g Point_Model_1\n');

% vertex position
for i = 1:nverts
    fprintf(fid, 'v %f %f %f\np %d\n', verts(i,:), i);
end
fprintf(fid, '# end of file\n');