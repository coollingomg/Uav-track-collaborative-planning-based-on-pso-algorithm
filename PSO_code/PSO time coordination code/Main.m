% 清空缓存
clear,clc,close all

%-------------------------定义五架飞机的起点和终点-------------------------%
startPos = [10 50  10;
            10 150 5;
            10 250 10;
            10 350 8;
            10 450 0;];
goalPos = [470, 420, 90];

%% -------------------------运行优化算法-------------------------%
% 定义相关变量
Gbest_Num = zeros(5, 5, 12);            % (五架飞机，五条路径，12维迭代)
fitness_beat_iters = zeros(5, 5, 600);
% 迭代5架飞机
for i=1:5
    mm=0.15;
    % 迭代每架飞机的所有路径
    for j=1:5
        % pso迭代
        mm=mm+0.05;
        if mm>=0.5
            mm=0.5;
        end
        [Gbest_Num(i, j, :), fitness_beat_iters(i, j, :)] = IDM_PSO(startPos(i,:), goalPos,mm);
        % 定义变量记录是否满足相关约束
        result = true;
        % 比对航迹点重复
        if j~=1
            % 检查新航迹与之前所有航迹的节点是否相同
            for kk = 1:j-1
                new_trajectory = reshape(Gbest_Num(i, j, :), 4, []);
                existing_trajectories = reshape(Gbest_Num(i, kk, :), 4, []);
                if compareTrajectories(new_trajectory, existing_trajectories)
                    fprintf('第 %d 架飞机的第 %d 航迹与第 %d 航迹至少有一个航机节点不相同\n',i,j, kk);
                else
                    result = false;
                    break;
                end
            end
            % 判断是否满足条件，否则跳出循环直接重新迭代
            if result~=true
                j=j-1;
                continue;
            end
        end
        % 比对航迹是否与前面的航迹保持一定的距离
        if i~=1
            for kk=1:i-1
                for jj=1:5
                    if checkDistance(Gbest_Num(i,j,:), Gbest_Num(kk,jj,:), 5, 450)
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
            % 判断是否满足条件，否则跳出循环直接重新迭代
            if result~=true
                j=j-1;
                continue;
            end
        end
    end
end

% 保存迭代的结果
save Main.mat Gbest_Num fitness_beat_iters;

%% 时间协同
% 加载运行的数据
load('Main.mat');

% 求出每条航迹的总长度
Distances=zeros(5, 5);
for i=1:5
    for j=1:5
        Distances(i,j)=getDistance(Gbest_Num(i,j,:),startPos(i,:),goalPos);
    end
end

% 求出时间范围
times=zeros(2,5,5);
times(1,:,:)=Distances/.65;     % 最小时间
times(2,:,:)=Distances/.45;     % 最大时间

% 求出每架飞机的时间并集
times_union=zeros(2,5);
for i=1:5
    for j=1:5
        if j==1
            times_union(1,i)=times(1,i,j);
            times_union(2,i)=times(2,i,j);
            fprintf('目前第 %d 架飞机的时间并集为[ %f , %f ] \n',i ,times_union(1,i),times_union(2,i));
        else
            if times(1,i,j)<=times_union(1,i)
                times_union(1,i)=times(1,i,j);
            end
            if times(2,i,j)>=times_union(2,i)
                times_union(2,i)=times(2,i,j);
            end
            fprintf('目前第 %d 架飞机的时间并集为[ %f , %f ] \n',i ,times_union(1,i),times_union(2,i));
        end
    end
end

% 所有飞机求时间交集
union_of_set=zeros(1,2);
result_union=true;
for i=1:5
    if i==1
        union_of_set(1)=times_union(1,i);
        union_of_set(2)=times_union(2,i);
        fprintf('目前的时间交集为[ %f , %f ] \n',union_of_set(1),union_of_set(2));
    else
        % 判断是否有交集
        if union_of_set(1) > times_union(2,i) ||  union_of_set(2) < times_union(1,i)
            result_union = false;
            break;
        end
        % 有交集了，才开始求交集
        if times_union(1,i)>=union_of_set(1)
            union_of_set(1)=times_union(1,i);
        end
        if times_union(2,i)<=union_of_set(2)
            union_of_set(2)=times(2,i);
        end
        fprintf('目前的时间交集为[ %f , %f ] \n',union_of_set(1),union_of_set(2));
    end
end

% 没有时间协同，抛出错误
if result_union==false
    error("所求的航迹不能够实现协同！");
end

% 有交集，并且求出了交集，分配航迹和速度
get_Gbest=zeros(5,12);
get_speed=zeros(5,1);
for i=1:5
    for j=1:5
        if times(1,i,j)<=union_of_set(1) && union_of_set(1)<= times(2,i,j)
            fprintf('选择了第 %d 架飞机的第 %d 条路径 \n',i,j);
            get_Gbest(i,:)=Gbest_Num(i,j,:);
            get_speed(i)=Distances(i,j)/union_of_set(1);
            break;
        end
    end
end

%% 画图展现
%------------------------1.迭代图-------------------------
figure
num_figure = fitness_beat_iters(1,1,:);
fitness_beat_iters_reshaped = reshape(num_figure, 1, []);
plot(1:length(fitness_beat_iters_reshaped),fitness_beat_iters_reshaped,'Color',[38 70 83]/255,'LineWidth',3);
hold on;
for i=2:5
    num_figure = fitness_beat_iters(i,1,:);
    fitness_beat_iters_reshaped = reshape(num_figure, 1, []);
    plot(1:length(fitness_beat_iters_reshaped),fitness_beat_iters_reshaped,'Color',[38 110 83]/255,'LineWidth',3);
end
grid on;
legend('IDM-PSO','FontSize',12,'FontName','Times New Roman');
xlabel('进化代数','FontSize',12);
ylabel('最优个体适应度','FontSize',12);
title('迭代适应度变化','FontSize',14);

%% ------------------------画二三维图展示-------------------------

% 设置随机种子为2，确保随机数生成器的初始状态是一致的，以便于生成的地图相同
rng(2);

% 威胁物定义
% 定义威胁物的坐标、半径等参数
menaceParams = [struct('start', [270, 200, 0], 'end', [270, 200, 100], 'radius', 20);
                struct('start', [170, 350, 0], 'end', [170, 350, 100], 'radius', 30);
                struct('start', [300, 300, 0], 'end', [300, 300, 100], 'radius', 25);
                struct('start', [350, 400, 0], 'end', [350, 400, 100], 'radius', 30);];

% 随机定义山峰地图
mapRange = [500,500,100];       % 地图长、宽、高范围
N=10;                           % 山峰个数，更改山峰个数同时要到defMap.m里更改山峰中心坐标
[X,Y,Z] = defMap(mapRange,N);   % 获取地图数据

% 画图
Draw_result_path(startPos, goalPos,X,Y,Z,get_Gbest,4,menaceParams);
