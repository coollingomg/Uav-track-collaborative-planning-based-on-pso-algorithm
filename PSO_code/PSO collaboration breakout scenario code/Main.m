% 清空缓存
clear,clc,close all

% 定义五架飞机的起点和终点
start_points = [10*10^3, 40*10^3,  50;
                10*10^3, 60*10^3,  50;
                10*10^3, 80*10^3,  50;
                10*10^3, 100*10^3, 50;
                10*10^3, 120*10^3, 50];
goalPos = [100*10^3, 79*10^3,  50;
           99*10^3,  79*10^3,  50;
           99*10^3,  80*10^3,  50;
           99*10^3,  81*10^3,  50;
           100*10^3, 81*10^3,  50];

% 威胁区定义
menaceParams = [struct('center', [50*10^3, 80*10^3],  'radius', 5*10^3);
                struct('center', [61*10^3, 61*10^3],  'radius', 5*10^3);
                struct('center', [60*10^3, 97*10^3],  'radius', 5*10^3);];

%% 运行优化算法
Gbest_Num = zeros(5, 5, 12);            % (五架飞机，五条路径，12维迭代)
fitness_best_iters = zeros(5, 5, 700);

% 迭代5架飞机
for i=1:5
    mm=0.10;        % 优化指标权重系数
    % 迭代每架飞机的所有路径
    j = 1;
    while j <= 5
        % 更新权重系数，但不超过0.5
        mm = min(mm + 0.05, 0.5);

        % 运行PSO算法
        [Gbest_Num(i, j, :), fitness_best_iters(i, j, :)] = IDM_PSO(start_points(i,:), goalPos(i,:), menaceParams, mm);

        % 默认满足约束条件
        result = true;

        % 检查航迹重复约束
        if j ~= 1
            for kk = 1:j-1
                new_trajectory = reshape(Gbest_Num(i, j, :), 1, []);
                existing_trajectories = reshape(Gbest_Num(i, kk, :), 1, []);
                if TrajectoryTool.compareTrajectories(new_trajectory, existing_trajectories, start_points(i,:), goalPos(i,:))
                    fprintf('第 %d 架飞机的第 %d 航迹与第 %d 航迹不相同\n', i, j, kk);
                else
                    result = false;
                    break;
                end
            end

            % 如果不满足重复约束，重新生成路径
            if result~=true
                continue;
            end
        end

        % 检查航迹距离约束
        if i > 1
            for kk=1:i-1
                for jj=1:5
                    trajectory1 = reshape(Gbest_Num(i, j, :), 1, []);
                    trajectory2 = reshape(Gbest_Num(kk, jj, :), 1, []);
                    if TrajectoryTool.checkSafeDistance(trajectory1, trajectory2, start_points, goalPos, i, kk, 100, 200*10^3)
                        continue;
                    else
                        result = false;
                        break;
                    end
                end

                % 不符合条件之间跳出循环
                if result~=true
                    break;
                end
            end
        end

        % 如果不满足距离约束，重新生成该路径
        if result~=true
            continue;
        else
            % 满足所有约束，继续生成下一条路径
            j = j + 1;
        end
    end
end

% 保存迭代的结果
save Main.mat Gbest_Num fitness_best_iters;

%% 时间协同
Distances=zeros(5, 5);      % 求出每条航迹的总长度
for i=1:5
    for j=1:5
        trajectory = reshape(Gbest_Num(i, j, :), 1, []);
        Distances(i,j)=TrajectoryTool.getDistance(trajectory, start_points(i,:), goalPos(i,:));
    end
end

% 求出时间范围
times=zeros(2,5,5);
times(1,:,:)=Distances./280;     % 最小时间
times(2,:,:)=Distances./200;     % 最大时间

% 计算每架飞机的时间并集
times_union = zeros(2, 5);
for i = 1:5
    times_union(1, i) = min(times(1, i, :));
    times_union(2, i) = max(times(2, i, :));
    fprintf('目前第 %d 架飞机的时间并集为[ %f , %f ] \n', i, times_union(1, i), times_union(2, i));
end

% 计算所有飞机的时间交集
union_of_set = [max(times_union(1, :)), min(times_union(2, :))];
% 检查时间交集是否存在
if union_of_set(1) > union_of_set(2)
    error("所求的航迹不能够实现协同！");
end
fprintf('所有飞机的时间交集为[ %f , %f ] \n', union_of_set(1), union_of_set(2));

% 分配航迹和速度
get_Gbest = zeros(5, 12);
get_speed = zeros(5, 1);
for i = 1:5
    for j = 1:5
        if times(1, i, j) <= union_of_set(1) && union_of_set(1) <= times(2, i, j)
            fprintf('选择了第 %d 架飞机的第 %d 条路径 \n', i, j);
            get_Gbest(i, :) = Gbest_Num(i, j, :);
            get_speed(i) = Distances(i, j) / union_of_set(1);
            break;
        end
    end
end

%% 画图
% 1.迭代图
figure
num_figure = fitness_best_iters(1,1,:);
fitness_beat_iters_reshaped = reshape(num_figure, 1, []);
plot(1:length(fitness_beat_iters_reshaped),fitness_beat_iters_reshaped,'LineWidth',2);
hold on;
for i=2:5
    num_figure = fitness_best_iters(i,1,:);
    fitness_beat_iters_reshaped = reshape(num_figure, 1, []);
    plot(1:length(fitness_beat_iters_reshaped),fitness_beat_iters_reshaped,'LineWidth',2);
end
grid on;
legend('IDM-PSO','FontSize',12,'FontName','Times New Roman');
xlabel('进化代数','FontSize',12);
ylabel('最优个体适应度','FontSize',12);
title('迭代适应度变化','FontSize',14);

% 2.路径图
DrawPaths.Draw_paths(start_points, goalPos, get_Gbest, 4, menaceParams);
