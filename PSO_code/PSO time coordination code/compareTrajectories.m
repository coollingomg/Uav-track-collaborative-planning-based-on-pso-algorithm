function result = compareTrajectories(trajectory1, trajectory2)
% -------------------------------------------------------------
% 比较两条航迹节点的函数
% -------------------------------------------------------------
% trajectory1: 第一条航迹的节点坐标矩阵
% trajectory2: 第二条航迹的节点坐标矩阵
% result: 若至少有一个节点不同，返回 true；否则返回 false
% -------------------------------------------------------------

% 默认情况下两条航迹的节点都相同
result = false;
for i = 1:size(trajectory1, 1)
    for j = 1:size(trajectory2, 1)
        % 检查每个坐标的绝对值之差是否小于1
        if all(abs(trajectory1(i, :) - trajectory2(j, :)) < 1)
            % 如果节点之间的差值小于1，则认为节点相同
            continue;
        else
            % 如果存在至少一个节点的差值大于等于1，则认为节点不同
            result = true;
            return;
        end
    end
end
end
