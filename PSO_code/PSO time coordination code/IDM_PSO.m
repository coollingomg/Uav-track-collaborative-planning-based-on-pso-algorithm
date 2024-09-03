function [Gbest, fitness_beat_iters] = IDM_PSO(startPos,goalPos,mm)
% -------------------------------------------------------------
% 改进的多种群粒子群算法
% -------------------------------------------------------------
% 改进策略：
% 整个种群分为三类：1优势群 2劣势群 3混合群 3类群在每次迭代过程中依据适应度动态组合
% 1.劣势群
% 高斯变异+混合更新
% 2.优势群
% 融合莱维飞行以及贪婪算法
% 3.混合群
% 融合动态权重因子和正余弦思想更新位置
% -------------------------------------------------------------


% 设置随机种子为2，确保随机数生成器的初始状态是一致的，以便于进行可重复的实验。
rng(2);

%----------------------------三维地图模型----------------------------%
% 定义路径规划的起点和终点坐标
% startPos = [10, 100, 10];
% goalPos = [470, 420, 60];

% 随机定义山峰地图
mapRange = [500,500,100];       % 地图长、宽、高范围
N=10;                           % 山峰个数，更改山峰个数同时要到defMap.m里更改山峰中心坐标
[X,Y,Z] = defMap(mapRange,N);   % 获取地图数据

% 创建图窗口
figure('Name','Map');
% 标记起点和终点
scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y');
hold on;
scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');

% 画山峰曲面
surf(X,Y,Z);        % 画曲面图
hold on;
shading interp;     % 使用插值方法进行着色，使得曲面上的颜色更加平滑
colormap summer;    % 内置颜色映射表，表示夏季风格的颜色，从深蓝渐变到浅黄

%-----------------------------威胁物定义--------------------------------%
% 定义威胁物的坐标、半径等参数
menaceParams = [struct('start', [270, 200, 0], 'end', [270, 200, 100], 'radius', 20);
                struct('start', [170, 350, 0], 'end', [170, 350, 100], 'radius', 30);
                struct('start', [300, 300, 0], 'end', [300, 300, 100], 'radius', 25);
                struct('start', [350, 400, 0], 'end', [350, 400, 100], 'radius', 30);];

% 绘制威胁物
alpha1=0.4;         % 不透明度
for i = 1:numel(menaceParams)
    menaceParam = menaceParams(i);
    [Xm, Ym, Zm] = myplotcylinder(menaceParam.start, menaceParam.end, menaceParam.radius);
    surf(Xm, Ym, Zm, 'facecolor', [2 48 71]/255, 'edgecolor', 'none', 'FaceAlpha', alpha1)
    hold on;
end

% 添加坐标轴信息
xlabel('x(m)','FontName','Times New Roman')
ylabel('y(m)','FontName','Times New Roman')
zlabel('z(m)','FontName','Times New Roman')

% 限制坐标轴的范围
xlim([0,500]);
ylim([0,500]);
hold off;

% 暂停执行一段时间，以提供足够的时间给图形界面更新显示
pause(0.001);

% 提取威胁物坐标和半径
menace_obj_R = [menaceParams.radius];
menace = zeros(4,2);
for i = 1:numel(menaceParams)
    menaceParam = menaceParams(i);
    startCoordinates = [menaceParam.start];
    menace(i,:) = startCoordinates(:, 1:2);
end

% 定义危险区域和无人机直径
S = 7;              % 危险区域
D = 2;              % 无人机直径

% 定义相关物理量及限制
M=5;                %无人机质量
g=9.8;              %重力加速度
Bmax = pi/2;        % 转弯角度
Umax=pi/2;          % 俯仰角度

%--------------------------------PSO参数定义--------------------------------%
pointnum = 3;               % xyz，三维的粒子群优化
dim = 4;                    % 经过节点个数，路径的四个节点
lenchrom = pointnum*dim;    % 位置向量的长度，即每个粒子的位置信息的总维度
sizepop = 30;               % 粒子数量
subsize = sizepop/3;        % 子族群粒子数量
maxgen = 600;               % 迭代次数

% 粒子速度界限
Vmax = repmat([20,20,20],[1,dim]);
Vmin = repmat([-20,-20,-20],[1,dim]);

% 学习因子
c1 = 1.49445;
c2 = 1.49445;
c3 = 1.49445;

% 动态权重参数
wmax = 0.9;
wmin = 0.5;

% 粒子位置界限
popmin = zeros(1,lenchrom);
popmax = [ones(1,dim)*mapRange(1) ones(1,dim)*mapRange(2) ones(1,dim)*mapRange(3)];

% 莱维飞行参数
beta = 3/2;
alpha = 0.01;               %步长控制因子

% 预定义空间
pop=zeros(sizepop,lenchrom);            % 粒子位置
V=zeros(sizepop,lenchrom);              % 粒子速度
fitness = zeros(sizepop, 1);            % 粒子适应度

%--------------------------------粒子群初始化--------------------------------%
for i=1:sizepop
    % 随机产生一个种群
    % 随机初始位置，通过unifrnd函数产生指定范围内的均匀分布的随机数
    x(1:dim) = unifrnd(popmin(1)+1, popmax(1), [1,dim]);
    x((dim+1):(2*dim)) = unifrnd(popmin(dim+1)+1, popmax(dim+1), [1,dim]);
    x((2*dim+1):(3*dim)) = unifrnd(popmin(end)+1, popmax(end), [1,dim]);
    % 对位置向量的每一部分进行排序
    x(1:dim) = sort(x(1:dim));
    x((dim+1):(2*dim)) = sort(x((dim+1):(2*dim)));
    x((2*dim+1):(3*dim)) = sort(x((2*dim+1):(3*dim)));

    % 初始化速度
    V(i,:)=Vmax.*rands(1,lenchrom); 
    % 加载单个粒子的位置到群
    pop(i,:) = x;

    %计算适应度
    fitness(i)=fun(pop(i,:),startPos,goalPos,X,Y,Z,menace,S,D,M,g,dim,menace_obj_R,mapRange,Bmax,Umax,mm); 
end

%--------------------------------加载迭代前的相关信息--------------------------------%
% 找到当前群体中最佳个体的适应度值和群体中最佳个体的索引
[fmin, ind] = min(fitness);
favg = mean(fitness);               % 计算所有个体适应度的均值
fGbest = fmin;                      % 全局最佳适应度值
Gbest = pop(ind, :);                % 全局最佳路径
fPbest = fitness;                   % 个体最佳适应度值
Pbest = pop;                        % 个体最佳路径

% 初始化每一代的最优适应度，用于画适应度迭代图
fitness_beat_iters = zeros(maxgen,1);

% 划分种群，从小到大
[Sort_f, index]=sort(fitness);  % 对适应度进行排序，并给出对应的索引
mid_ = sizepop/2;               % 中位数，总粒子数量的一半
top_pop = index(1:mid_);        % 提取适应度排序后前一半的索引（索引对应未排序的矩阵）
bottom_pop = index(mid_+1:end); % 提取后一半

% 随机从这两个中各取部分，这段代码是随机排列top_pop和bottom_pop的索引
top_sel = randperm(mid_,sizepop/2);
bot_sel = randperm(mid_,sizepop/2);

% 组合三个子族群，这些都是fitness的索引
mix_swarm = [top_pop(top_sel(1:subsize/2)) bottom_pop(bot_sel(1:subsize/2))]; % 混合
top_swarm = top_pop(top_sel(subsize/2+1:end));                                % 优势
bot_swarm = bottom_pop(top_sel(subsize/2+1:end));                             % 劣势

%--------------------------------迭代寻优--------------------------------%
for i=1:maxgen
    % 粒子状态更新
    for j=1:sizepop
        % 惯性权重非线性递减（使用cos函数非线性化）
        w = wmin+(wmax-wmin)*cos(pi*i/maxgen);
        
        % 优势群更新机制
        if sum(j == top_swarm) == 1
            % 传统更新速度和位置
            % 速度更新
            V(j,:) = w*V(j,:) + c1*rand*(Pbest(j,:) - pop(j,:)) + c2*rand*(Gbest - pop(j,:));
            % 速度边界约束
            V(j,find(V(j,:)>Vmax)) = Vmax(find(V(j,:)>Vmax));
            V(j,find(V(j,:)<Vmin)) = Vmin(find(V(j,:)<Vmin));
    
            % 位置更新
            pop(j,:) = pop(j,:) + V(j,:);
            % 位置边界约束
            pop(j,find(pop(j,:)>popmax)) = popmax(find(pop(j,:)>popmax));
            pop(j,find(pop(j,:)<popmin)) = popmin(find(pop(j,:)<popmin));

            % 适应度值更新
            fitness(j,:) = fun(pop(j,:),startPos,goalPos,X,Y,Z,menace,S,D,M,g,dim,menace_obj_R,mapRange,Bmax,Umax,mm);

            % 莱维飞行（个人理解，传统更新飞完之后，再莱维飞一次）
            % 但是这个是尝试的飞行，如果飞完了结果没前面好，就不跟新位置，相当于重新飞回原来的位置
            Levy_step = levy(lenchrom,beta);
            X1 = pop(j,:) + alpha * Levy_step;

            % 越界处理，将粒子位置限制在搜索范围中
            for k=1:lenchrom
                if X1(k) > popmax(k)
                    X1(k) = popmax(k);
                end
                if X1(k) < popmin(k)
                    X1(k) = popmin(k);
                end
            end
            % 在使用莱维飞行后，更新适应度值进行后面的贪婪策略
            f1 = fun(X1,startPos,goalPos,X,Y,Z,menace,S,D,M,g,dim,menace_obj_R,mapRange,Bmax,Umax,mm);
            % 贪婪策略，比较莱维飞行后的适应度值，如果比之前的适应度好，则保留
            if f1 < fitness(j,:)
                pop(j,:) = X1;
            end
        
        % 劣势群更新机制
        elseif sum(j == bot_swarm) == 1
            % 随机选择最优组成pmix（生成混合粒子），numb为1到sizepop随机抽取lenchrom个的矩阵
            numb = unidrnd(sizepop,[1 lenchrom]);
            for ii=1:lenchrom
                pmix(ii) = Pbest(numb(ii),ii);
            end
            
            % 速度更新
            V(j,:) = w*V(j,:) + c1*rand*(Pbest(j,:) - pop(j,:)) + c2*rand*(Gbest - pop(j,:)) + c3*rand*(pmix-pop(j,:));
            % 速度边界约束
            V(j,find(V(j,:)>Vmax)) = Vmax(find(V(j,:)>Vmax));
            V(j,find(V(j,:)<Vmin)) = Vmin(find(V(j,:)<Vmin));
    
            % 位置更新
            pop(j,:)=pop(j,:)+V(j,:);
            % 位置边界约束
            pop(j,find(pop(j,:)>popmax))=popmax(find(pop(j,:)>popmax));
            pop(j,find(pop(j,:)<popmin))=popmin(find(pop(j,:)<popmin));

            % 适应度值更新
            fitness(j,:) = fun(pop(j,:),startPos,goalPos,X,Y,Z,menace,S,D,M,g,dim,menace_obj_R,mapRange,Bmax,Umax,mm); 

            % 高斯变异，变异之后适应度比之前好就变异，不然不变异
            if rand > (1/2*(1+atan(i/maxgen)*4/pi))         % 变异概率判断，不是每次都变异
                % 随机位置变异
                X1 = pop(j,:).*(1+randn(1,lenchrom));
                % 边界限制
                X1(find(X1>popmax)) = popmax(find(X1>popmax));
                X1(find(X1<popmin)) = popmin(find(X1<popmin));
                % 更新变异后的适应度
                fNEW = fun(X1,startPos,goalPos,X,Y,Z,menace,S,D,M,g,dim,menace_obj_R,mapRange,Bmax,Umax,mm);
                % 贪婪策略，比较变异后的适应度值，如果比之前的适应度好，则保留
                if fNEW < fitness(j,:)
                    pop(j,:) = X1;
                    fitness(j,:) = fNEW;
                end
            end
            
        % 混合群更新机制    
        elseif sum(j==mix_swarm)==1
            % 速度更新
            V(j,:) = w*V(j,:) + 2*cos(pi*i/2/maxgen)*rand*(Pbest(j,:) - pop(j,:)) + 2*sin(pi*i/2/maxgen)*rand*(Gbest - pop(j,:));
            % 速度边界约束
            V(j,find(V(j,:)>Vmax)) = Vmax(find(V(j,:)>Vmax));
            V(j,find(V(j,:)<Vmin)) = Vmin(find(V(j,:)<Vmin));
    
            % 位置更新
            pop(j,:)=pop(j,:)+V(j,:);
            % 位置边界约束
            pop(j,find(pop(j,:)>popmax))=popmax(find(pop(j,:)>popmax));
            pop(j,find(pop(j,:)<popmin))=popmin(find(pop(j,:)<popmin));

            % 适应度值更新
            fitness(j,:) = fun(pop(j,:),startPos,goalPos,X,Y,Z,menace,S,D,M,g,dim,menace_obj_R,mapRange,Bmax,Umax,mm); 
        end
    end
    
    % 迭代之后，重新更具适应度划分种群
    [Sort_f, index] = sort(fitness);
    top_pop = index(1:sizepop/2);
    bottom_pop = index(sizepop/2+1:end);
    % 从这两个中各取部分
    top_sel = randperm(sizepop/2,sizepop/2);
    bot_sel = randperm(sizepop/2,sizepop/2);
    % 三个子族群
    mix_swarm = [top_pop(top_sel(1:subsize/2)) bottom_pop(bot_sel(1:subsize/2))]; % 混合
    top_swarm = top_pop(top_sel(subsize/2+1:end));                                % 优势
    bot_swarm = bottom_pop(top_sel(subsize/2+1:end));                             % 劣势
    
    % 更新种群的最优信息
    for j=1:sizepop
        % 个体最优更新
        if fitness(j) < fPbest(j)
            Pbest(j,:) = pop(j,:);                      % 个体最佳路径更新
            fPbest(j) = fitness(j);                     % 个体最佳适应度值更新
        end
        % 群体最优更新
        if fitness(j) < fGbest
            Gbest = pop(j,:);                           % 全局最佳路径更新
            fGbest = fitness(j);                        % 全局最佳适应度值更新
        end
    end

    % 记录不断迭代的全局最佳适应度值
    fitness_beat_iters(i)=fGbest;     
    
    % 在命令行窗口显示每一代的信息
    disp(['IDM-PSO-----第' num2str(i) '代:' '最优适应度 = ' num2str(fGbest)]);
    
    % 绘制路径规划的图形，包括起点、终点、山峰地形、威胁物、无人机路径等
    Draw_path(startPos,goalPos,X,Y,Z, Gbest, dim,menaceParams);
    % 暂停执行一段时间，以提供足够的时间给图形界面更新显示
    pause(0.001);
end

%---------------------------------------------------------------%
% 使用 save 函数将指定的变量保存到PSO.mat文件中，保存了以下变量：
%---------------------------------------------------------------%
% Gbest：算法得到的最佳路径或最佳解
% fitness_beat_iters：算法每一代的最优适应度值组成的向量
%---------------------------------------------------------------%
% save IDM_PSO.mat Gbest fitness_beat_iters
end

