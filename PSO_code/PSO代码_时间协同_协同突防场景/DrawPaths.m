classdef DrawPaths
    methods (Static)
        function Draw_path(startPos, goalPos, Gbest, dim, menaceParams)
        % --------------------------------------------------------------
        % Draw_path - 绘制路径规划的图形，包括起点、终点、威胁物、无人机路径等（单条）
        % 输入参数:
        %   startPos - 起点坐标 [x, y, z]
        %   goalPos  - 终点坐标 [x, y, z]
        %   Gbest    - 最佳路径的坐标数组
        %   dim      - 路径节点数
        %   menaceParams - 威胁区信息的结构体数组
        % --------------------------------------------------------------
            figure(1);
        
            % 画起点和终点
            p1 = scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y');
            hold on;
            p2 = scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');
        
            % 威胁物绘制
            for i = 1:numel(menaceParams)
                DrawPaths.drawForbiddenZone(menaceParams(i).center, menaceParams(i).radius);
                hold on;
            end
        
            % 添加坐标轴信息
            xlabel('x/km','FontName','Times New Roman');
            ylabel('y/km','FontName','Times New Roman');
            zlabel('z/m','FontName','Times New Roman');
        
            % 路径平滑
            [X_seq, Y_seq, Z_seq] = DrawPaths.smoothPath(startPos, goalPos, Gbest, dim);
        
            % 绘制无人机路线
            p3 = plot3(X_seq, Y_seq, Z_seq, 'LineWidth', 2);
            hold off;
            grid on;
            
            % 添加图例
            legend([p1, p2, p3],{'起点','终点','无人机路线'},"LineWidth",2);
        end

        function Draw_paths(startPos, goalPos, Gbest, dim, menaceParams)
        % --------------------------------------------------------------
        % Draw_paths - 绘制结果路径的函数（多条）
        % 输入参数:
        %   startPos - 起点坐标数组 [x, y, z] (n x 3)
        %   goalPos  - 终点坐标 [x, y, z]
        %   Gbest    - 最佳路径的坐标数组 (n x 3*dim)
        %   dim      - 路径节点数
        %   menaceParams - 威胁区信息的结构体数组
        % --------------------------------------------------------------
            % 三维路线图
            figure(2)
            
            % 绘制起点
            scatter3(startPos(:,1), startPos(:,2), startPos(:,3), 100, 'bs', 'MarkerFaceColor', 'y');
            hold on;
        
            % 绘制威胁区
            for i = 1:numel(menaceParams)
                DrawPaths.drawForbiddenZone(menaceParams(i).center, menaceParams(i).radius);
            end
        
            % 添加坐标轴信息
            xlabel('x(m)','FontName','Times New Roman')
            ylabel('y(m)','FontName','Times New Roman')
            zlabel('z(m)','FontName','Times New Roman')
        
            % 路径绘制
            numStartPoints = size(startPos, 1);
            p = gobjects(numStartPoints, 1);
            for i = 1:numStartPoints
                [X_seq, Y_seq, Z_seq] = DrawPaths.smoothPath(startPos(i, :), goalPos(i, :), Gbest(i, :), dim);
                p(i)=plot3(X_seq, Y_seq, Z_seq, 'LineWidth', 2);
            end

            % 标记终点
            plot3(goalPos(3,1), goalPos(3,2), goalPos(3,3), 'p', 'MarkerFaceColor', 'red', 'MarkerSize', 20);
        
            hold off;
            grid on;
        
            % 添加图例
            legend(p, arrayfun(@(i) ['UAV', num2str(i)], 1:numStartPoints, 'UniformOutput', false));
            % 添加标题
            title('三维路线图')
        % -----------------------------------------------------------------------------------------------
            % 二维路线图
            figure(3)
            
            % 绘制起点
            scatter(startPos(:,1), startPos(:,2), 100, 'bs', 'MarkerFaceColor', 'y');
            hold on;
        
            % 绘制威胁物
            for i = 1:numel(menaceParams)
                DrawPaths.drawForbiddenZone2D(menaceParams(i).center, menaceParams(i).radius);
            end
        
            % 添加坐标轴信息
            xlabel('x/m','FontName','Times New Roman')
            ylabel('y/m','FontName','Times New Roman')
            xlim([0, 110*10^3]);
            ylim([30*10^3, 130*10^3]);

            % 路径绘制
            for i = 1:numStartPoints
                [X_seq, Y_seq, ~] = DrawPaths.smoothPath(startPos(i, :), goalPos(i, :), Gbest(i, :), dim);
                p(i)=plot(X_seq, Y_seq, 'LineWidth', 2);
            end

            % 标记终点
            plot(goalPos(3,1), goalPos(3,2), 'p', 'MarkerFaceColor', 'red', 'MarkerSize', 15);

            hold off;
        
            % 添加图例
            legend(p, arrayfun(@(i) ['UAV', num2str(i)], 1:numStartPoints, 'UniformOutput', false));
            % 添加图像标题
            title('路线俯视图');
        end

        function [X_seq, Y_seq, Z_seq] = smoothPath(startPos, goalPos, Gbest, dim)
        % --------------------------------------------------------------
        % smoothPath - 对路径进行三次样条插值平滑处理
        % 输入参数:
        %   startPos - 起点坐标 [x, y, z]
        %   goalPos  - 终点坐标 [x, y, z]
        %   Gbest    - 最佳路径的坐标数组
        %   dim      - 路径节点数
        % 输出参数:
        %   X_seq, Y_seq, Z_seq - 平滑处理后的路径坐标数组
        % --------------------------------------------------------------
            % 获取原始路径节点
            x_seq = [startPos(1), Gbest(1:dim), goalPos(1)];
            y_seq = [startPos(2), Gbest(dim+1:2*dim), goalPos(2)];
            z_seq = [startPos(3), Gbest(2*dim+1:3*dim), goalPos(3)];
            
            numOriginalPoints = length(x_seq);                      % 获取原始路径节点的数量
            originalIndices = linspace(0, 1000, numOriginalPoints); % 生成长度为 numOriginalPoints 的等差数列，表示原始节点的归一化位置
            interpIndices = linspace(0, 1000, 1000);                % 生成长度为 1000 的等差数列，表示插值后路径的归一化位置
            
            % 使用spline函数对路径进行三次样条插值
            X_seq = spline(originalIndices, x_seq, interpIndices);
            Y_seq = spline(originalIndices, y_seq, interpIndices);
            Z_seq = spline(originalIndices, z_seq, interpIndices);
        end

        function drawForbiddenZone(center, radius)
        % --------------------------------------------------------------
        % drawForbiddenZone - 绘制禁止区域的函数
        % 输入参数:
        %   center - 禁止区域的中心 [x, y]
        %   radius - 禁止区域的半径
        % --------------------------------------------------------------
            % 生成极坐标角度
            theta = linspace(0, 2*pi, 100);
            % 生成高度范围
            heights = linspace(0, 800, 100);
            
            % 初始化坐标矩阵
            x = zeros(length(heights), length(theta));
            y = zeros(length(heights), length(theta));
            z = zeros(length(heights), length(theta));
            
            % 计算禁止区域的三维坐标
            for i = 1:length(heights)
                x(i, :) = center(1) + radius * cos(theta);
                y(i, :) = center(2) + radius * sin(theta);
                z(i, :) = heights(i);
            end
            
            % 绘制彩色禁止区域曲面
            surf(x, y, z, 'FaceColor', 'interp', 'EdgeColor', 'none', 'DisplayName', 'Forbidden Zone');
            
            % 添加颜色映射
            colormap(parula);
        end

        function drawForbiddenZone2D(center, radius)
        % --------------------------------------------------------------
        % drawForbiddenZone2D - 绘制二维禁止区域的函数
        % 输入参数:
        %   center - 禁止区域的中心 [x, y]
        %   radius - 禁止区域的半径
        % --------------------------------------------------------------
            % 生成圆上的点
            theta = linspace(0, 2*pi, 100);

            x_circle = radius * cos(theta) + center(1);
            y_circle = radius * sin(theta) + center(2);
            
            % 绘制黑色线条圆和红色实心圆
            plot(x_circle, y_circle, 'k', 'LineWidth', 1, 'DisplayName', 'Forbidden Zone');
            fill(x_circle, y_circle, 'r', 'FaceAlpha', 0.5, 'DisplayName', 'Forbidden Zone');
        end
    end
end
