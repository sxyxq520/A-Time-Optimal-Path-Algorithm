function [ IC ] = Inertia_transform( ICB, M, T )

%计算相对于给定框架g原点的惯性矩阵
%输入百分比:
% ICB:相对于物体坐标系B中质心的惯性矩阵，B为
%位于原点。
% M:质量
% t:g和b之间的变换矩阵，T = T_G^B.
%如果点p在框架B中有坐标y，在框架B中有坐标x
%帧G，然后[x；1]= T[y；1]
%输出:
% IC:相对于框架G中G原点的惯性矩阵。
A=T(1:3,1:3);
d=T(1:3,4);
IC1 = Inertia_body_frame(ICB,A);
IC = Inertia_parallel_axis(IC1,M,-d);


end

