% 清空运行环境
clc;
clear;
close all;

% 设置随机种子为2，确保随机数生成器的初始状态是一致的，以便于进行可重复的实验。
rng(2);

%% 三维地图模型
% 定义路径规划的起点和终点坐标
startPos = [10, 400, 10];
goalPos = [470, 420, 60];

% 随机定义山峰地图
mapRange = [500,500,100];       % 地图长、宽、高范围
N=10;                           % 山峰个数，更改山峰个数同时要到defMap.m里更改山峰中心坐标
[X,Y,Z] = defMap(mapRange,N);   % 10个山峰

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

% 威胁物定义；更新威胁物坐标时Draw_path.m里的也要更新
alpha1=0.4;         % 不透明度

% 定义威胁物的坐标、半径等参数
menaceParams = [
    struct('start', [270, 200, 0], 'end', [270, 200, 100], 'radius', 20);
    struct('start', [170, 350, 0], 'end', [170, 350, 100], 'radius', 30);
    struct('start', [300, 300, 0], 'end', [300, 300, 100], 'radius', 25);
    struct('start', [350, 400, 0], 'end', [350, 400, 100], 'radius', 30);
];

% 绘制威胁物
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

%% 粒子群参数初始化
maxgen = 600;                % 迭代次数
sizepop = 30;               % 粒子数量
pointnum = 3;               % xyz，三维的粒子群优化
dim =4;                    % 经过节点个数，路径的四个节点
lenchrom = pointnum*dim;    % 位置向量的长度，即每个粒子的位置信息的总维度。

% 线性递减惯性权重
ws=0.9;
we=0.4;
% 学习率
c1 = 2;                     % 认知权重（个体学习率）
c2 = 2;                     % 社会权重（社会学习率）

% 粒子位置界限
popmin = zeros(1,lenchrom);
popmax = [ones(1,dim)*mapRange(1) ones(1,dim)*mapRange(2) ones(1,dim)*mapRange(3)];

% 粒子速度界限
Vmax = repmat([20,20,20],[1,dim]);
Vmin = repmat([-20,-20,-20],[1,dim]);

% 预定义空间
pop=zeros(sizepop,lenchrom);            % 粒子位置
V=zeros(sizepop,lenchrom);              % 粒子速度
fitness = zeros(sizepop, 1);            % 适应度

%% 产生初始粒子和速度
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
    fitness(i)=fun(pop(i,:),startPos,goalPos,X,Y,Z,menace,S,D,M,g,dim,menace_obj_R,mapRange,Bmax,Umax); 
end

%% 个体极值和群体极值
% 找到当前群体中最佳个体的适应度值和群体中最佳个体的索引
[bestfitness, bestindex]=min(fitness);
zbest=pop(bestindex,:);             % 全局最佳路径
gbest=pop;                          % 个体最佳路径
fitnessgbest=fitness;               % 个体最佳适应度值
fitnesszbest=bestfitness;           % 全局最佳适应度值

% 初始化每一代的最优适应度，用于画适应度迭代图
fitness_beat_iters = zeros(maxgen,1);

%% 迭代寻优
for i=1:maxgen
    % 粒子状态更新
    for j=1:sizepop
        % 惯性系数线性递减
         w=ws-(ws-we)*i/maxgen;

        % 速度更新
        V(j,:) = w*V(j,:) + c1*rand*(gbest(j,:) - pop(j,:)) + c2*rand*(zbest - pop(j,:));
        % 速度边界约束
        V(j,find(V(j,:)>Vmax)) = Vmax(find(V(j,:)>Vmax));
        V(j,find(V(j,:)<Vmin)) = Vmin(find(V(j,:)<Vmin));

        % 位置更新
        pop(j,:)=pop(j,:)+V(j,:);
        % 位置边界约束
        pop(j,find(pop(j,:)>popmax))=popmax(find(pop(j,:)>popmax));
        pop(j,find(pop(j,:)<popmin))=popmin(find(pop(j,:)<popmin));

        % 适应度值更新
        fitness(j)=fun(pop(j,:),startPos,goalPos,X,Y,Z,menace,S,D,M,g,dim,menace_obj_R,mapRange,Bmax,Umax); 
    end
    
    % 粒子最佳状态记录更新
    for j=1:sizepop
        % 个体最优更新
        if fitness(j) < fitnessgbest(j)
            gbest(j,:) = pop(j,:);                  % 个体最佳路径更新
            fitnessgbest(j) = fitness(j);           % 个体最佳适应度值更新
        end
        % 群体最优更新
        if fitness(j) < fitnesszbest
            zbest = pop(j,:);                       % 全局最佳路径更新
            fitnesszbest = fitness(j);              % 全局最佳适应度值更新
        end
    end

    % 记录不断迭代的全局最佳适应度值
    fitness_beat_iters(i)=fitnesszbest; 
    
    % 在命令行窗口显示每一代的信息
    disp(['PSO-----第' num2str(i) '代:' '最优适应度 = ' num2str(fitnesszbest)]);
    
%     % 绘制路径规划的图形，包括起点、终点、山峰地形、威胁物、无人机路径等
%     Draw_path(startPos,goalPos,X,Y,Z, zbest, dim,menaceParams);
%     % 暂停执行一段时间，以提供足够的时间给图形界面更新显示
%     pause(0.001);
end

%% 结果展示
% 三维、二维图
Draw_result_path(startPos,goalPos,X,Y,Z,zbest,dim,menaceParams)

% 画适应度迭代图
figure('Name','最佳适应度变化图');
plot(fitness_beat_iters,'LineWidth',2);
xlabel('迭代次数');
ylabel('最优适应度');

%{
使用 save 函数将指定的变量保存到PSO.mat文件中，保存了以下变量：
---------------------------------------------------------------
startPos：起点的位置信息
goalPos：终点的位置信息
zbest1：算法得到的最佳路径或最佳解
X、Y、Z：地图的三维坐标点
dim：轨迹节点数量
S 和 D：危险区域和无人机直径
yy1：算法每一代的最优适应度值组成的向量
%}
zbest1=zbest;
yy1=fitness_beat_iters;
save PSO.mat startPos goalPos zbest1 X Y Z dim S D yy1
