classdef TrajectoryTool
    methods (Static)
        function distance = getDistance(trajectory, startPos, goalPos)
        % -------------------------------------------------------------
        % getDistance 计算航迹的总长度
        % -------------------------------------------------------------
        % 参数:
        %   trajectory: 1x3n 的数组，表示航迹中的 (x, y, z) 节点
        %   startPos: 1x3 数组，表示起点的 (x, y, z) 坐标
        %   goalPos: 1x3 数组，表示终点的 (x, y, z) 坐标
        % -------------------------------------------------------------
        % 返回值:
        %   distance: 航迹的总长度
        % -------------------------------------------------------------
            % 确保输入参数的有效性
            if mod(length(trajectory), 3) ~= 0
                error('trajectory 数组的长度必须是 3 的倍数');
            end

            % 对航迹进行插值平滑
            num_points = length(trajectory) / 3;
            [X_seq, Y_seq, Z_seq] =DrawPaths.smoothPath(startPos, goalPos, trajectory, num_points);

            % 计算相邻插值点之间的距离并求和
            dx = diff(X_seq);  % 计算路径在 x 上相邻点的差值
            dy = diff(Y_seq);  % 计算路径在 y 上相邻点的差值
            dz = diff(Z_seq);  % 计算路径在 z 上相邻点的差值
            distance = sum(sqrt(dx.^2 + dy.^2 + dz.^2));  % 计算路径长度
        end
        
        function isSafe = checkSafeDistance(trajectory1, trajectory2, startPos, goalPos, i, j, Ds, Dc)
        % -------------------------------------------------------------
        % checkSafeDistance 检查两架飞机之间的距离是否满足要求
        % -------------------------------------------------------------
        % 参数:
        %   trajectory1: 1x3n 的数组，表示第一架飞机的航迹
        %   trajectory2: 1x3n 的数组，表示第二架飞机的航迹
        %   startPos: 1x3 数组，表示起点的 (x, y, z) 坐标
        %   goalPos: 1x3 数组，表示终点的 (x, y, z) 坐标 
        %   i: 为当前第i架飞机
        %   j：为前1：i-1架飞机
        %   Ds: 最小安全距离阈值
        %   Dc: 最大通信距离阈值
        % -------------------------------------------------------------
        % 返回值:
        %   isSafe: 布尔值，表示是否满足最小安全距离要求
        % -------------------------------------------------------------
            % 对航迹进行插值平滑
            num_points1 = length(trajectory1) / 3;
            num_points2 = length(trajectory2) / 3;
            [X_seq1, Y_seq1, Z_seq1] =DrawPaths.smoothPath(startPos(i, :), goalPos(i, :), trajectory1, num_points1);
            [X_seq2, Y_seq2, Z_seq2] =DrawPaths.smoothPath(startPos(j, :), goalPos(j, :), trajectory2, num_points2);

            % 计算两架飞机的距离
            distances = sqrt((X_seq1 - X_seq2).^2 + (Y_seq1 - Y_seq2).^2 + (Z_seq1 - Z_seq2).^2);

            % 检查距离是否在所需范围内
            isSafe = all(distances > Ds & distances < Dc);
            
            % 输出结果
            if isSafe
                disp('两架飞机之间的距离在要求范围内。');
            else
                disp('两架飞机之间的距离不满足要求。');
            end
        end

        function within = compareTrajectories(trajectory1, trajectory2, startPos, goalPos)
        % -------------------------------------------------------------
        % compareTrajectories 比较两条航迹是否完全重合
        % -------------------------------------------------------------
        % 参数:
        %   trajectory1: 1x3n 的数组，表示第一条航迹的节点
        %   trajectory2: 1x3n 的数组，表示第二条航迹的节点
        % -------------------------------------------------------------
        % 返回值:
        %   within: 布尔值，表示两条航迹满足不重合（真值）
        % -------------------------------------------------------------
            % 对航迹进行插值平滑
            num_points1 = length(trajectory1) / 3;
            num_points2 = length(trajectory2) / 3;
            [X_seq1, Y_seq1, Z_seq1] =DrawPaths.smoothPath(startPos, goalPos, trajectory1, num_points1);
            [X_seq2, Y_seq2, Z_seq2] =DrawPaths.smoothPath(startPos, goalPos, trajectory2, num_points2);

            % 计算两架飞机的距离
            distances = sqrt((X_seq1 - X_seq2).^2 + (Y_seq1 - Y_seq2).^2 + (Z_seq1 - Z_seq2).^2);

            % 检查两条航迹距离是否在所需范围内
            within = any(distances > 100);
        end
    end
end
