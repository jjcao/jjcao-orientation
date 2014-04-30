function [ normal ] = compute_lsp_normal( pointset )
%FINDMSPN Summary of this function goes here
%   Detailed explanation goes here
% �����붥�㼯�ϵ���С����ƽ��ĵ�λ����
% pointset ��һ��n-by-3����
x = pointset(:,1);
y = pointset(:,2);
z = pointset(:,3);

[A,B,C]=compute_plane_fit(x,y,z);
normal = [A,B,C]/sqrt(A*A+B*B+C*C);

end

