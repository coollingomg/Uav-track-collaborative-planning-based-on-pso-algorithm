function [Gbest, fitness_beat_iters, params] = PSO(startPos, goalPos, menaceParams, IDM_params)
% -------------------------------------------------------------
% 改进的多种群粒子群算法
% -------------------------------------------------------------
    % 所有参数定义到params结构体中
    params = createParams(startPos, goalPos, menaceParams);
    % 加载之前定义的地图，两个算法的避免随机性
    params.map = IDM_params.map;
    
    % 粒子群初始化
    [pop, V, fitness, fitPbest, Pbest, fitGbest, Gbest] = initializeParticles(params);

    % 初始化每一代的最优适应度
    fitness_beat_iters = zeros(params.maxgen,1);
    
    %--------------------------------迭代寻优--------------------------------%
    for i=1:params.maxgen
        % 惯性权重非线性递减（使用cos函数非线性化）
        w = params.wmin + (params.wmax - params.wmin) * cos(pi * i / params.maxgen);
    
        % 粒子状态更新
        for j=1:params.sizepop
            [pop, V, fitness] = updateSwarm(params, pop, V, Pbest, Gbest, fitness, j, w);
        end
    
        % 更新种群的最优信息
        [fitPbest, Pbest, fitGbest, Gbest] = updateBestInfo(pop, fitness, fitPbest, Pbest, fitGbest, Gbest);

        % 记录不断迭代的全局最佳适应度值
        fitness_beat_iters(i)=fitGbest;
        
        % 在命令行窗口显示每一代的信息
        disp(['IDM-PSO-----第' num2str(i) '代:' '最优适应度 = ' num2str(fitGbest)]);

        % 绘制路径规划的图形，包括起点、终点、山峰地形、威胁物、无人机路径等
        DrawPaths.Draw_path(startPos, goalPos, Gbest, params);
        % 暂停执行一段时间，以提供足够的时间给图形界面更新显示
        pause(0.001);
    end
end

function params = createParams(startPos, goalPos, menaceParams)
% -------------------------------------------------------------
% 创建参数结构体
% -------------------------------------------------------------
    params.startPos = startPos;
    params.goalPos = goalPos;
    params.menaceParams = menaceParams;

    % 提取威胁物坐标和半径
    params.menace_obj_R = [menaceParams.radius];
    params.menace = zeros(numel(menaceParams), 2);
    for i = 1:numel(menaceParams)
        menaceParam = menaceParams(i);
        startCoordinates = [menaceParam.center];
        params.menace(i, :) = startCoordinates(:, 1:2);
    end

    % 定义危险区域和无人机直径
    params.S = 7;  % 危险区域（m）
    params.D = 2;  % 无人机直径（m）

    % 定义相关物理量及限制
    params.M = 5;        % 无人机质量
    params.g = 9.8;      % 重力加速度
    params.Bmax = pi/2;  % 转弯角度
    params.Umax = pi/2;  % 俯仰角度

    % PSO参数
    params.pointnum = 3;                             % xyz，三维的粒子群优化
    params.dim = 4;                                  % 经过节点个数，路径的四个节点
    params.lenchrom = params.pointnum * params.dim;  % 位置向量的长度，即每个粒子的位置信息的总维度
    params.sizepop = 30;                             % 粒子数量
    params.subsize = params.sizepop / 3;             % 子族群粒子数量
    params.maxgen = 600;                             % 迭代次数

    % 粒子速度界限
    params.velocityLimits = [20, 20, 20];            % 定义每个维度的速度界限
    params.Vmax = [repmat(params.velocityLimits(1), 1, params.dim), ...
                   repmat(params.velocityLimits(2), 1, params.dim), ...
                   repmat(params.velocityLimits(3), 1, params.dim)];
    params.Vmin = -params.Vmax;

    % 学习因子
    params.c1 = 1.49445;
    params.c2 = 1.49445;

    % 动态权重参数
    params.wmax = 0.9;
    params.wmin = 0.5;

    % 粒子位置界限
    params.mapRange = [500, 500, 100];
    params.popMin = zeros(1, params.lenchrom);
    params.popMax = [repmat(params.mapRange(1), 1, params.dim), ...
                     repmat(params.mapRange(2), 1, params.dim), ...
                     repmat(params.mapRange(3), 1, params.dim)];
end

function [pop, V, fitness, fitPbest, Pbest, fitGbest, Gbest] = initializeParticles(params)
% -------------------------------------------------------------
% 初始化粒子群
% -------------------------------------------------------------
    pop = zeros(params.sizepop, params.lenchrom);   % 粒子位置
    V = zeros(params.sizepop, params.lenchrom);     % 粒子速度
    fitness = zeros(params.sizepop, 1);             % 粒子适应度

    % % 初始化种群
    % for i = 1:params.sizepop
    %     x1 = 10.^(log10(1) + (log10(params.mapRange(1)) - log10(1)) * rand(1, params.dim));
    %     x2 = 10.^(log10(1) + (log10(params.mapRange(2)) - log10(1)) * rand(1, params.dim));
    %     x3 = 10.^(log10(1) + (log10(params.mapRange(3)) - log10(1)) * rand(1, params.dim));
    %     x = [sort(x1), sort(x2), sort(x3)];
    % 
    %     % 初始化速度
    %     V(i, :) = params.Vmax .* randn(1, params.lenchrom);
    %     % 初始化粒子
    %     pop(i, :) = x;
    %     % 计算适应度
    %     fitness(i) = fun(pop(i, :), params);
    % end

    % 初始化种群
    for i=1:params.sizepop
        % 随机初始位置，通过unifrnd函数产生指定范围内的均匀分布的随机数
        x(1:params.dim) = unifrnd(params.popMin(1)+1, params.popMax(1), [1,params.dim]);
        x((params.dim+1):(2*params.dim)) = unifrnd(params.popMin(params.dim+1)+1, params.popMax(params.dim+1), [1,params.dim]);
        x((2*params.dim+1):(3*params.dim)) = unifrnd(params.popMin(end)+1, params.popMax(end), [1,params.dim]);
        % 对位置向量的每一部分进行排序
        x(1:params.dim) = sort(x(1:params.dim));
        x((params.dim+1):(2*params.dim)) = sort(x((params.dim+1):(2*params.dim)));
        x((2*params.dim+1):(3*params.dim)) = sort(x((2*params.dim+1):(3*params.dim)));
    
        % 初始化速度
        V(i,:)=params.Vmax.*rands(1,params.lenchrom); 
        % 加载单个粒子的位置到群
        pop(i,:) = x;
    
        %计算适应度
        fitness(i)=fun(pop(i, :), params);
    end

    [fmin, ind] = min(fitness);
    fitGbest = fmin;        % 全局最佳适应度值
    Gbest = pop(ind, :);    % 全局最佳路径
    fitPbest = fitness;     % 个体最佳适应度值
    Pbest = pop;            % 个体最佳路径
end

function [pop, V, fitness] = updateSwarm(params, pop, V, Pbest, Gbest, fitness, j, w)
% -------------------------------------------------------------
% 粒子群更新
% -------------------------------------------------------------
    % 速度更新
    V(j, :) = w * V(j, :) + params.c1 * rand * (Pbest(j, :) - pop(j, :)) + params.c2 * rand * (Gbest - pop(j, :));
    V(j, :) = max(min(V(j, :), params.Vmax), params.Vmin);
    % 位置更新
    pop(j, :) = pop(j, :) + V(j, :);
    pop(j, :) = max(min(pop(j, :), params.popMax), params.popMin);
    % 适应度值更新
    fitness(j, :) = fun(pop(j, :), params);
end

function [fitPbest, Pbest, fitGbest, Gbest] = updateBestInfo(pop, fitness, fitPbest, Pbest, fitGbest, Gbest)
% -------------------------------------------------------------
% 更新种群的最优信息
% -------------------------------------------------------------
    % 更新种群的最优信息
    better_Pbest = fitness < fitPbest;
    Pbest(better_Pbest, :) = pop(better_Pbest, :);      % 个体最佳路径更新  
    fitPbest(better_Pbest) = fitness(better_Pbest);     % 个体最佳适应度值更新 

    % 更新全局最优
    [fitGbest_, idx] = min(fitPbest);
    if fitGbest_ < fitGbest
        fitGbest = fitGbest_;                           % 全局最佳适应度值更新
        Gbest = pop(idx, :);                            % 全局最佳路径更新
    end
end
