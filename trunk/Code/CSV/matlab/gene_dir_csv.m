function [datadir, outdatadir] = gene_dir(database)
% datadir -- test model path
% outdatadir -- the OBNE  file path


base = '../../OBNE/dataOBNE/';
outpuOben='../result';
datadir=[base database '/'];

if ~isdir(datadir)
    error('No model direction')
end

outdatadir=outpuOben;
if ~isdir(outdatadir)
    mkdir(spdir);
end