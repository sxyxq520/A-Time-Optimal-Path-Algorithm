function [ IC ] = Inertia_transform( ICB, M, T )

%��������ڸ������gԭ��Ĺ��Ծ���
%����ٷֱ�:
% ICB:�������������ϵB�����ĵĹ��Ծ���BΪ
%λ��ԭ�㡣
% M:����
% t:g��b֮��ı任����T = T_G^B.
%�����p�ڿ��B��������y���ڿ��B��������x
%֡G��Ȼ��[x��1]= T[y��1]
%���:
% IC:����ڿ��G��Gԭ��Ĺ��Ծ���
A=T(1:3,1:3);
d=T(1:3,4);
IC1 = Inertia_body_frame(ICB,A);
IC = Inertia_parallel_axis(IC1,M,-d);


end

