classdef DrawPaths
    methods (Static)
        function Draw_path(startPos, goalPos, Gbest, params)
        % --------------------------------------------------------------
        % Draw_path - 绘制路径规划的图形，包括起点、终点、威胁物、无人机路径等（单条）
        % 输入参数:
        %   startPos - 起点坐标 [x, y, z]
        %   goalPos  - 终点坐标 [x, y, z]
        %   Gbest    - 最佳路径的坐标数组
        %   params   - 参数结构体
        % --------------------------------------------------------------
            figure(1);
        
            % 获取相关参数
            dim = params.dim;                   % 经过节点的个数
            menaceParams = params.menaceParams; % 威胁区信息的结构体数组
            X = params.map.X;                   % 地图x
            Y = params.map.Y;                   % 地图y
            Z = params.map.Z;                   % 地图z

            % 画起点和终点
            p1 = scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y');
            hold on;
            p2 = scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');

            % 画山峰曲面
            surf(X, Y, Z);      % 画曲面图
            shading interp;     % 使用插值方法进行着色，使得曲面上的颜色更加平滑
            colormap summer;    % 内置颜色映射表，表示夏季风格的颜色，从深蓝渐变到浅黄
        
            % 威胁物绘制
            for i = 1:numel(menaceParams)
                DrawPaths.drawForbiddenZone(menaceParams(i).center, menaceParams(i).radius);
                hold on;
            end
        
            % 添加坐标轴信息
            xlabel('x/km','FontName','Times New Roman');
            ylabel('y/km','FontName','Times New Roman');
            zlabel('z/m','FontName','Times New Roman');
            % 限制坐标轴的范围
            xlim([0,500]);
            ylim([0,500]);

            % 路径平滑
            [X_seq, Y_seq, Z_seq] = DrawPaths.smoothPath(startPos, goalPos, Gbest, dim);
        
            % 绘制无人机路线
            p3 = plot3(X_seq, Y_seq, Z_seq, 'LineWidth', 2);
            hold off;
            grid on;

            % 添加图例
            legend([p1, p2, p3],{'起点','终点','无人机路线'},"LineWidth",2);
        end

        function Draw_result_paths(startPos, goalPos, Gbest, params)
        % --------------------------------------------------------------
        % Draw_result_paths - 绘制结果路径的函数
        % 输入参数:
        %   startPos - 起点坐标数组 [x, y, z] (n x 3)
        %   goalPos  - 终点坐标 [x, y, z]
        %   Gbest    - 最佳路径的坐标数组 (n x 3*dim)
        %   dim      - 路径节点数
        %   X,Y,Z    - 地图
        %   menaceParams - 威胁区信息的结构体数组
        % --------------------------------------------------------------
            % 三维路线图
            figure;

            % 获取相关参数
            dim = params.dim;                   % 经过节点的个数
            menaceParams = params.menaceParams; % 威胁区信息的结构体数组
            X = params.map.X;                   % 地图x
            Y = params.map.Y;                   % 地图y
            Z = params.map.Z;                   % 地图z
            
            % 画起点和终点
            p1 = scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y');
            hold on;
            p2 = scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');

            % 画山峰曲面
            surf(X, Y, Z);      % 画曲面图
            shading interp;     % 使用插值方法进行着色，使得曲面上的颜色更加平滑
            colormap summer;    % 内置颜色映射表，表示夏季风格的颜色，从深蓝渐变到浅黄
        
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
            % 添加标题
            title('三维路线图')
        % -----------------------------------------------------------------------------------------------
            % 二维路线图
            figure;
            
            % 画起点和终点
            p1 = scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y');
            hold on;
            p2 = scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');

            % 画山峰曲面
            surf(X, Y, Z);      % 画曲面图
            shading interp;     % 使用插值方法进行着色，使得曲面上的颜色更加平滑
            colormap summer;    % 内置颜色映射表，表示夏季风格的颜色，从深蓝渐变到浅黄
        
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

            % 添加图像标题
            title('路线俯视图');

            % 看俯视图
            view(2)
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
            originalIndices = linspace(0, 100, numOriginalPoints);  % 生成长度为 numOriginalPoints 的等差数列，表示原始节点的归一化位置
            interpIndices = linspace(0, 100, 100);                  % 生成长度为 1000 的等差数列，表示插值后路径的归一化位置
            
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
            heights = linspace(0, 100, 100);
            
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
            surf(x, y, z, 'facecolor', [2 48 71]/255, 'edgecolor', 'none', 'FaceAlpha', 0.4);
            
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

        function [X,Y,Z] = defMap(mapRange, N)
        % -------------------------------------------------------------
        % 说明：定义生成山峰地图的函数defMap
        % -------------------------------------------------------------
        % 参数：
        % mapRange：是一个包含地图长、宽、高范围的向量
        % N：是山峰的个数
        % 函数返回三个矩阵 X,Y,Z，分别表示网格点的 x、y、z 坐标
        % -------------------------------------------------------------
            % 山峰特征参数的初始化
            peaksInfo = struct;                 % 初始化山峰特征信息结构体
            peaksInfo.center = [];              % 山峰中心
            peaksInfo.range = [];               % 山峰区域
            peaksInfo.height = [];              % 山峰高度
            peaksInfo = repmat(peaksInfo,N,1);  % 复制N个山峰结构体构成矩阵
            
            % 预定义山峰的中心坐标
            center = [445 120;50 70; 150 450;170 150;400 450;350 270;70 300 ;250 400;300 100;170 390];
            
            % 遍历山峰，为每个山峰生成随机的高度和范围，并分配中心点
            for i = 1:N
                peaksInfo(i).center = center(i,:);
                peaksInfo(i).height = mapRange(3) * (rand*0.8+0.2);
                peaksInfo(i).range = mapRange * 0.1 * (rand*0.7+0.3);
            end
            
            % 计算山峰曲面值
            % 初始化一个矩阵 peakData 存储山峰的曲面值
            peakData = zeros(mapRange(1), mapRange(2));
            % 使用高斯函数计算每个点的山峰曲面值
            for x = 1:mapRange(1)
                for y = 1:mapRange(2)
                    sum = 0;
                    for k = 1:N
                        h_i = peaksInfo(k).height;
                        x_i = peaksInfo(k).center(1);
                        y_i = peaksInfo(k).center(2);
                        x_si = peaksInfo(k).range(1);
                        y_si = peaksInfo(k).range(2);
                        sum = sum + h_i * exp(-((x-x_i)/x_si)^2 - ((y-y_i)/y_si)^2);
                    end
                    peakData(x, y) = sum;
                end
            end
    
            % 返回生成的地图数据点信息
            [x , y] = meshgrid(1:mapRange(1), 1:mapRange(2));
            X = x;
            Y = y;
            Z = peakData;
        end

        function plot_evolution_curve(fitnessPSO, fitnessIDM_PSO)
        % -----------------------------------------------------------------
        % 说明：绘制PSO和IDM-PSO算法的适应度随迭代次数变化的曲线
        % -----------------------------------------------------------------
        % 参数：
        % fitnessPSO：向量，表示PSO算法在每一代中的最优个体适应度
        % fitnessIDM_PSO：向量，表示IDM-PSO算法在每一代中的最优个体适应度
        % -----------------------------------------------------------------
            % 检查输入向量的长度是否一致
            if length(fitnessPSO) ~= length(fitnessIDM_PSO)
                error('输入向量 fitnessPSO 和 fitnessIDM_PSO 必须具有相同的长度');
            end
            
            % 绘制第一个数据集
            plot(1:length(fitnessPSO), fitnessPSO, 'Color', [233 196 107]/255, 'LineWidth', 2);
            hold on;
            
            % 绘制第二个数据集
            plot(1:length(fitnessIDM_PSO), fitnessIDM_PSO, 'Color', [38 70 83]/255, 'LineWidth', 2);
            
            % 添加网格
            grid on;
            
            % 添加图例
            legend('PSO', 'IDM-PSO', 'FontSize', 12, 'FontName', 'Times New Roman');
            
            % 添加坐标轴标签
            xlabel('进化代数', 'FontSize', 12);
            ylabel('最优个体适应度', 'FontSize', 12);
            
            % 添加标题
            title('进化曲线对比', 'FontSize', 14);
            
            % 保持绘图
            hold off;
        end
    end
end