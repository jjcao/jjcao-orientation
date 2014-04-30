function [vertex,face,normal, uv, sphparam] = read_mesh(file)

% read_mesh - read data from OFF, PLY, SMF or WRL file.
%
%   [vertex,face] = read_mesh(filename);
%   [vertex,face] = read_mesh;      % open a dialog box
%
%   'vertex' is a '3 x nb.vert' array specifying the position of the vertices.
%   'face' is a '3 x nb.face' array specifying the connectivity of the mesh.
%
%   Supported file extensions are: .off, .ply, .wrl, .obj, .m, .gim.
%
%   Copyright (c) 2007 Gabriel Peyre

if nargin==0
    [f, pathname] = uigetfile({'*.off;*.obj;*.pwn'},'Pick a file');
    file = [pathname,f];
end

i = strfind(file,'.');
ext = file(i(length(i))+1:end);


switch lower(ext)
    case 'off'
        [vertex,face,normal] = read_off(file);
    case 'obj'
        [vertex,normal] = read_pcloud_obj(file);
        face=[];
    case 'pwn'
        [vertex,normal] = read_pwn(file);   
        face=[];
    otherwise
        error('Unknown extension.');
end