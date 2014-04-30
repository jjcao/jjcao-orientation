function [datadir, outdatadir] = gene_dir(database)
% datadir -- test model path
% outdatadir -- the OBNE  file path


base = '../../Models/';
outpuOben=['dataOBNE/'];
datadir=[base database '/'];

if ~isdir(datadir)
    error('No model direction')
end

outdatadir=outpuOben;
if ~isdir(outdatadir)
    mkdir(spdir);
end