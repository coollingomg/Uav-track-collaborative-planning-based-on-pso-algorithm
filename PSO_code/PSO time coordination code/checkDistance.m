function distance_check = checkDistance(trajectory1, trajectory2, Ds, Dc)
% -------------------------------------------------------------
% 检查两架飞机之间的距离是否满足要求
% -------------------------------------------------------------
% 参数:
%   trajectory1: 第一架飞机的航迹
%   trajectory2: 第二架飞机的航迹
%   Ds: 最小安全距离阈值
%   Dc: 最大通信距离阈值
% -------------------------------------------------------------
% 返回值:
%   distance_check: 若两架飞机之间的距离在要求范围内，返回 true；否则返回 false
% -------------------------------------------------------------

% 使用 spline 函数对航迹进行三次样条插值，生成更多的点
k = length(trajectory1) / 3;     % 获取航迹节点的数量
i_seq = linspace(0, 100, k);     % 生成长度为 k 的归一化位置序列
xx_seq = linspace(0, 100, 100);  % 生成长度为 100 的归一化位置序列，用于插值后的路径
X_seq = spline(i_seq, trajectory1(1:4), xx_seq);   % 对 x 坐标进行插值
Y_seq = spline(i_seq, trajectory1(5:8), xx_seq);   % 对 y 坐标进行插值
Z_seq = spline(i_seq, trajectory1(9:12), xx_seq);  % 对 z 坐标进行插值
trajectory1 = [X_seq, Y_seq, Z_seq];
X_seq = spline(i_seq, trajectory2(1:4), xx_seq);   % 对 x 坐标进行插值
Y_seq = spline(i_seq, trajectory2(5:8), xx_seq);   % 对 y 坐标进行插值
Z_seq = spline(i_seq, trajectory2(9:12), xx_seq);  % 对 z 坐标进行插值
trajectory2 = [X_seq, Y_seq, Z_seq];

% 将航迹重塑为矩阵形式
trajectory1 = reshape(trajectory1, 100, []);
trajectory2 = reshape(trajectory2, 100, []);

% 计算两架飞机的距离
distances = sqrt(sum((trajectory1 - trajectory2).^2, 2));

% 检查距离是否在所需范围内
if all(distances > Ds) && all(distances < Dc)
    distance_check = true;
    disp('两架飞机之间的距离在要求范围内。');
else
    distance_check = false;
    disp('两架飞机之间的距离不满足要求。');
end
end
