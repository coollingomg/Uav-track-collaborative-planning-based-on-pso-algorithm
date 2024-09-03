function distance = getDistance(trajectory,startPos,goalPos)
% -------------------------------------------------------------
% 计算航迹的总长度
% -------------------------------------------------------------
% 参数:
%   trajectory: 飞机的航迹
%   startPos: 飞机的航迹起点
%   goalPos: 飞机的航迹终点
% -------------------------------------------------------------
% 返回值:
%   distance: 航迹的总长度
% -------------------------------------------------------------


% 使用 spline 函数对航迹进行三次样条插值，生成更多的点
k = length(trajectory) / 3+2;    % 获取航迹节点的数量
i_seq = linspace(0, 100, k);     % 生成长度为 k 的归一化位置序列
xx_seq = linspace(0, 100, 100);  % 生成长度为 100 的归一化位置序列，用于插值后的路径
X_seq = spline(i_seq, [startPos(1),reshape(trajectory(1:4),1,[]),goalPos(1)], xx_seq);   % 对 x 坐标进行插值
Y_seq = spline(i_seq, [startPos(2),reshape(trajectory(5:8),1,[]),goalPos(2)], xx_seq);   % 对 y 坐标进行插值
Z_seq = spline(i_seq, [startPos(3),reshape(trajectory(9:12),1,[]),goalPos(3)], xx_seq);  % 对 z 坐标进行插值

% 计算两架飞机的距离
dx = diff(X_seq);                         % 计算路径在 x 上相邻点的差值
dy = diff(Y_seq);                         % 计算路径在 y 上相邻点的差值
dz = diff(Z_seq);                         % 计算路径在 z 上相邻点的差值
distance = sum(sqrt(dx.^2 + dy.^2 + dz.^2));

end
