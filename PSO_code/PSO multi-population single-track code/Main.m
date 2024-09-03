% 清空缓存
clear,clc;

% 定义飞机的起点和终点
start_points = [10, 400, 10];
goalPos = [470, 420, 60];

% 威胁区定义
menaceParams = [struct('center', [270, 200],  'radius', 20);
                struct('center', [170, 350],  'radius', 30);
                struct('center', [300, 300],  'radius', 25);
                struct('center', [350, 400],  'radius', 30);];

% 分别运行两个算法得到结果并保存数据
[IDM_Gbest, IDM_fitness_beat_iters, IDM_params] = IDM_PSO(start_points, goalPos, menaceParams);
save IDM_PSO.mat IDM_Gbest IDM_fitness_beat_iters IDM_params

[Gbest, fitness_beat_iters, params] = PSO(start_points, goalPos, menaceParams, IDM_params);
save PSO.mat Gbest fitness_beat_iters params

%% 加载运行的数据，画图分析
load('PSO.mat');
load('IDM_PSO.mat');

% 画图展现轨迹
DrawPaths.Draw_result_paths(start_points, goalPos, IDM_Gbest, IDM_params);
DrawPaths.Draw_result_paths(start_points, goalPos, Gbest, params);
% 绘制迭代适应度值
DrawPaths.plot_evolution_curve(fitness_beat_iters, IDM_fitness_beat_iters);
