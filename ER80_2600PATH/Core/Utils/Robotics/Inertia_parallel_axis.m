function [ IR ] = Inertia_parallel_axis( IC,M,d )

%转动惯量的%平行轴定理
%将惯性矩阵从重心移动到参考点
%计算相对于参考点R的惯性矩阵，而不是
%质量中心百分比
%输入百分比:
% IC:相对于质心的惯性矩阵
% M:质量
% d:从质心到参考点R的矢量
%输出:
% IR:相对于参考点R的惯性矩阵
% author: Yu Zhao, yzhao334@berkeley.edu
%% useful function
hat=@(w)[0      -w(3)   w(2);...
         w(3)   0       -w(1);...
         -w(2)  w(1)    0];
IR = IC - M * (hat(d))^2;



end

