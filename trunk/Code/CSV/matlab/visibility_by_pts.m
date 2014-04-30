function P = visibility_by_pts(P, sk_filename)
%
% create: 2010-11-02
% by: JJCAO
%
%% setting
IS_DEBUG = 1;
SHOW_RESULT = 1;
SAVE_RESULT=1;
if ~exist('sk_filename','var')
    path('toolbox',path);
    sk_filename='../data/Moai_contract_nn(10)_WL(7.212904)_WH(2.000000).mat';
    load(sk_filename,'P');
end

% camposi = [1,0,0;];%1
% camposi = [1,0,0;-1,0,0;0,1,0;0,-1,0;];%4
%% camposi = [1,0,0;-1,0,0;0,1,0;0,0,1;0,0,-1;];% 5 face: 6 - -Y axis
%camposi = [1,0,0;-1,0,0;0,1,0;0,-1,0;0,0,1;0,0,-1;];% 6 face
camposi = [1,0,0;-1,0,0;0,1,0;0,-1,0;0,0,1;0,0,-1;% 6 face
    1,-1,-1;  1,-1,1; 1,1,-1; 1,1,1; 
    -1,-1,-1;-1,-1,1;-1,1,-1; -1,1,1% 8 corner
    ];% 14
% camposi = [0,1,0; 0,1,1; 0,0,1; 0,-1,1; 0,-1,0; 0,-1,-1; 0,0,-1; 0,1,-1;%x=0 plane
%            1,0,0; 1,0,1;        -1,0,1; -1,0,0; -1,0,-1;         1,0,-1;
%                   1,1,0;        1,-1,0;         -1,-1,0;         -1,1,0;
%     ];%18
% camposi = [0,1,0; 0,1,1; 0,0,1; 0,-1,1; 0,-1,0; 0,-1,-1; 0,0,-1; 0,1,-1;%x=0 plane
%            1,0,0; 1,0,1;        -1,0,1; -1,0,0; -1,0,-1;         1,0,-1;
%                   1,1,0;        1,-1,0;         -1,-1,0;         -1,1,0;
%     1,-1,-1;  1,-1,1; 1,1,-1; 1,1,1; 
%     -1,-1,-1;-1,-1,1;-1,1,-1; -1,1,1% 8 corner
%     ];%26
contrno = 1;% only use the contraction of 1st time.
param = 2;% large value for large view,5 is too large
ncam = size(camposi,1);
fprintf('visibility_by_pts: %d view \n', ncam);

P.contrno = contrno;
p= [P.pts; P.cpts{contrno}];
tic
P.tag = zeros(P.npts, 2);
file_view=fopen('../result/result_viewpoint.txt','wt');%add by liujian
fprintf(file_view,'%d\n',ncam);%add by liujian
for i=1:ncam
    visiblePtInds=HPR(p,camposi(i,:),param);
    vp = visiblePtInds(visiblePtInds>P.npts)-P.npts;%visible points of contracted points
    vptp = visiblePtInds(visiblePtInds<(P.npts+1));%visible points of original points
    fprintf(file_view,'%d %d %d\n',camposi(i,:));%add by liujian
    fprintf(file_view,'%d\n',size(vptp,1));%add by liujian
    for i1=1:size(vptp,1) %add by liujian
        fprintf(file_view,'%d ',vptp(i1,1));%add by liujian
    end%add by liujian
    fprintf(file_view,'\n');%add by liujian
    P.tag(vp,1) = P.tag(vp,1) + 1;
    P.tag(vptp,2) = P.tag(vptp,2) + 1;    
end
fclose(file_view);%add by liujian
toc

