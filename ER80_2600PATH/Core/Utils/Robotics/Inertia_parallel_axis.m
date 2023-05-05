function [ IR ] = Inertia_parallel_axis( IC,M,d )

%ת��������%ƽ���ᶨ��
%�����Ծ���������ƶ����ο���
%��������ڲο���R�Ĺ��Ծ��󣬶�����
%�������İٷֱ�
%����ٷֱ�:
% IC:��������ĵĹ��Ծ���
% M:����
% d:�����ĵ��ο���R��ʸ��
%���:
% IR:����ڲο���R�Ĺ��Ծ���
% author: Yu Zhao, yzhao334@berkeley.edu
%% useful function
hat=@(w)[0      -w(3)   w(2);...
         w(3)   0       -w(1);...
         -w(2)  w(1)    0];
IR = IC - M * (hat(d))^2;



end

