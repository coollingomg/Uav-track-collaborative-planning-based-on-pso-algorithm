function [Gbest, fitness_beat_iters, params] = IDM_PSO(startPos, goalPos, menaceParams)
% -------------------------------------------------------------
% 改进的多种群粒子群算法
% -------------------------------------------------------------
    % 所有参数定义到params结构体中
    params = createParams(startPos, goalPos, menaceParams);
    
    % 粒子群初始化
    [pop, V, fitness, fitPbest, Pbest, fitGbest, Gbest] = initializeParticles(params);

    % 初始化每一代的最优适应度
    fitness_beat_iters = zeros(params.maxgen,1);
    
    % 划分种群，从小到大
    [mix_swarm, top_swarm, bot_swarm] = divideSwarm(fitness, params);
    
    %--------------------------------迭代寻优--------------------------------%
    for i=1:params.maxgen
        % 惯性权重非线性递减（使用cos函数非线性化）
        w = params.wmin + (params.wmax - params.wmin) * cos(pi * i / params.maxgen);
    
        % 粒子状态更新
        for j=1:params.sizepop
            if ismember(j, top_swarm)       % 优势群更新机制
                [pop, V, fitness] = updateTopSwarm(params, pop, V, Pbest, Gbest, fitness, j, w);
            elseif ismember(j, bot_swarm)   % 劣势群更新机制
                [pop, V, fitness] = updateBotSwarm(params, pop, V, Pbest, Gbest, fitness, j, w, i);
            elseif ismember(j, mix_swarm)   % 混合群更新机制
                [pop, V, fitness] = updateMixSwarm(params, pop, V, Pbest, Gbest, fitness, j, w, i);
            end
        end
        
        % 迭代之后，重新划分种群
        [mix_swarm, top_swarm, bot_swarm] = divideSwarm(fitness, params);
    
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
    params.c3 = 1.49445;

    % 动态权重参数
    params.wmax = 0.9;
    params.wmin = 0.5;

    % 粒子位置界限
    params.mapRange = [500, 500, 100];
    params.popMin = zeros(1, params.lenchrom);
    params.popMax = [repmat(params.mapRange(1), 1, params.dim), ...
                     repmat(params.mapRange(2), 1, params.dim), ...
                     repmat(params.mapRange(3), 1, params.dim)];

    % 随机定义带地形的地图
    [params.map.X, params.map.Y, params.map.Z] = DrawPaths.defMap(params.mapRange, 10);   % 10个山峰

    % 莱维飞行参数
    params.beta = 3/2;
    params.alpha = 0.1;  % 步长控制因子
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

function [mix_swarm, top_swarm, bot_swarm] = divideSwarm(fitness, params)
% -------------------------------------------------------------
% 划分种群
% -------------------------------------------------------------
    % 重新根据适应度划分种群
    [~, index] = sort(fitness);
    mid_ = params.sizepop / 2;
    top_pop = index(1:mid_);
    bottom_pop = index(mid_ + 1:end);

    % 随机打乱
    top_sel = randperm(mid_, params.sizepop / 2);
    bot_sel = randperm(mid_, params.sizepop / 2);

    % 分配种群的序号
    mix_swarm = [top_pop(top_sel(1:params.subsize / 2)), bottom_pop(bot_sel(1:params.subsize / 2))];
    top_swarm = top_pop(top_sel(params.subsize / 2 + 1:end));
    bot_swarm = bottom_pop(bot_sel(params.subsize / 2 + 1:end));
end

function [pop, V, fitness] = updateTopSwarm(params, pop, V, Pbest, Gbest, fitness, j, w)
% -------------------------------------------------------------
% 优势群更新机制
% -------------------------------------------------------------
    % 速度更新
    V(j, :) = w * V(j, :) + params.c1 * rand * (Pbest(j, :) - pop(j, :)) + params.c2 * rand * (Gbest - pop(j, :));
    V(j, :) = max(min(V(j, :), params.Vmax), params.Vmin);
    % 位置更新
    pop(j, :) = pop(j, :) + V(j, :);
    pop(j, :) = max(min(pop(j, :), params.popMax), params.popMin);
    % 适应度值更新
    fitness(j, :) = fun(pop(j, :), params);

    % 莱维飞行
    Levy_step = levy(params.lenchrom, params.beta);
    X1 = pop(j, :) + params.alpha * Levy_step;
    % 越界处理
    X1 = max(min(X1, params.popMax), params.popMin);
    % 在使用莱维飞行后，更新适应度值进行后面的贪婪策略
    f1 = fun(X1, params);
    if f1 < fitness(j, :)
        pop(j, :) = X1;
    end
end

function [pop, V, fitness] = updateBotSwarm(params, pop, V, Pbest, Gbest, fitness, j, w, i)
% -------------------------------------------------------------
% 劣势群更新机制
% -------------------------------------------------------------
    % 随机选择最优组成pmix（生成组合粒子）
    numb = unidrnd(params.sizepop, [1 params.lenchrom]);
    pmix = arrayfun(@(ii) Pbest(numb(ii), ii), 1:params.lenchrom);
    % 速度更新
    V(j, :) = w * V(j, :) + params.c1 * rand * (Pbest(j, :) - pop(j, :)) + params.c2 * rand * (Gbest - pop(j, :)) + params.c3 * rand * (pmix - pop(j, :));
    V(j, :) = max(min(V(j, :), params.Vmax), params.Vmin);
    % 位置更新
    pop(j, :) = pop(j, :) + V(j, :);
    pop(j, :) = max(min(pop(j, :), params.popMax), params.popMin);
    % 适应度值更新
    fitness(j, :) = fun(pop(j, :), params);

    % 高斯变异
    if rand > (1 / 2 * (1 + atan(i / params.maxgen) * 4 / pi))
        X1 = pop(j, :) .* (1 + randn(1, params.lenchrom));
        X1 = max(min(X1, params.popMax), params.popMin);
        fNEW = fun(X1, params);
        if fNEW < fitness(j, :)
            pop(j, :) = X1;
            fitness(j, :) = fNEW;
        end
    end
end

function [pop, V, fitness] = updateMixSwarm(params, pop, V, Pbest, Gbest, fitness, j, w, i)
% -------------------------------------------------------------
% 混合群更新机制
% -------------------------------------------------------------
    % 速度更新
    V(j, :) = w * V(j, :) + 2 * cos(pi * i / 2 / params.maxgen) * rand * (Pbest(j, :) - pop(j, :)) + 2 * sin(pi * i / 2 / params.maxgen) * rand * (Gbest - pop(j, :));
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

function Levy_step = levy(dimension, beta)
% -------------------------------------------------------------
% 计算莱维飞行步长
% dimension : 维度
% beta : 莱维飞行的指数参数
% -------------------------------------------------------------
    % 计算方差alpha_u
    alpha_u = (gamma(1 + beta) * sin(pi * beta / 2) / (gamma(((1 + beta) / 2) * beta * 2^((beta - 1) / 2))))^(1 / beta);
    % 方差alpha_v
    alpha_v = 1;

    % u和v服从正态分布
    u = normrnd(0, alpha_u^2, [1, dimension]);
    v = normrnd(0, alpha_v^2, [1, dimension]);
    
    % 计算Levy步长
    Levy_step = 0.01 * u ./ (abs(v).^(1 / beta));
end
