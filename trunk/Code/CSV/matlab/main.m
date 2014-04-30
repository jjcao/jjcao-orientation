%% Step 0: read file (point cloud & local feature size if possible), and
clear;clc;close all;
path('toolbox',path);
%读取点云数据
database = 'closeby'; % sharpFeature,closeby,noise,nonuniform,topology
[imdir, spdir] = gene_dir_csv(database);
imnames=dir([imdir '*' 'off']);
for i=1:length(imnames)
    filename=imnames(i).name;
    file=filename(1:end-4);
    format=imnames(i).name(end-3:end);
    tic
    P.filename = [file format];% point set
    [P.pts,P.faces, P.normal1]=read_off([imdir filename]);
    % write_xyz('out.xyz',P.pts,[]);
    P.npts = size(P.pts,1);
    P.pts = GS.normalize(P.pts);
    [P.bbox, P.diameter] = GS.compute_bbox(P.pts);
    disp(sprintf('read point set:'));
    toc

    [P outfilename]= contraction(P);
    tic
    P = visibility_by_pts(P, outfilename);
    toc
end