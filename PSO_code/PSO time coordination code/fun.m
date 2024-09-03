function fitness = fun(pop, startPos, goalPos, X, Y, Z, menace, S, D, M, g, dim, menace_obj_R, mapRange, Bmax, Umax,mm)
% -------------------------------------------------------------
% 适应度函数，用于计算粒子群算法中每个粒子的适应度值
% 适应度规则：判断与地形是否相交、与威胁物距离、俯仰角和转角是否满足，不满足的用罚函数处理
% -------------------------------------------------------------
% 输入参数:
% pop: 当前粒子的位置，表示飞行路径的节点
% startPos: 起始点的坐标 [x, y, z]
% goalPos: 目标点的坐标 [x, y, z]
% X, Y, Z: 地形的网格坐标
% menace: 威胁物的坐标，每行表示一个威胁物的位置 [x, y, z]
% S: 安全距离
% D: 毁灭区域半径
% M: 高度代价权重
% g: 重力加速度
% dim: 经过节点的个数
% menace_obj_R: 威胁物的影响半径
% mapRange: 地图的范围 [x_max, y_max, z_max]
% Bmax: 最大转弯角度
% Umax: 最大俯仰角度
% -------------------------------------------------------------
% 输出参数:
% fitness: 粒子的适应度值
% -------------------------------------------------------------

%% 使用三次样条插值，对路径进行平滑
% 获取插值点数据，即路径的离散节点
x_seq=[startPos(1), pop(1:dim), goalPos(1)];
y_seq=[startPos(2), pop(dim+1:2*dim), goalPos(2)];
z_seq=[startPos(3), pop(2*dim+1:3*dim), goalPos(3)];

% 使用 spline 函数对路径进行三次样条插值，生成更多的点，使路径更加平滑
k = length(x_seq);                            % 获取原始路径节点的数量
i_seq = linspace(0,100,k);                    % 生成长度为 k 的等差数列，表示原始节点的归一化位置
xx_seq = linspace(0,100,100);                 % 生成长度为 100 的等差数列，表示插值后路径的归一化位置
yy_seq = linspace(0,100,100);                 % 生成长度为 100 的等差数列，表示插值后路径的归一化位置
zz_seq = linspace(0,100,100);                 % 生成长度为 100 的等差数列，表示插值后路径的归一化位置
X_seq = spline(i_seq,x_seq,xx_seq);           % 利用三次样条进行插值，得到插值预测的100个路径节点
Y_seq = spline(i_seq,y_seq,yy_seq);           % 利用三次样条进行插值，得到插值预测的100个路径节点
Z_seq = spline(i_seq,z_seq,zz_seq);           % 利用三次样条进行插值，得到插值预测的100个路径节点
path = [X_seq', Y_seq', Z_seq'];              % 得到三维路径的矩阵

%% 约束判断
% 检查生成的路径是否与地形相交，如果相交，flag 设置为1，表示违反了地形约束
flag = 0;                                     % 初始化标志位，表示路径与约束（地形，威胁区）不相交
for i = 2:size(path,1)
    x = path(i,1);                            % 获取平滑路径中当前点的 x 坐标
    y = path(i,2);                            % 获取平滑路径中当前点的 y 坐标
    z_interp = interp2(X,Y,Z,x,y);            % 使用二维插值（三次样条插值）获取路径点在地形表面上的插值高度
    if path(i,3) < z_interp                   % 如果平滑路径中当前点的 z 坐标小于插值高度（即在地形表面下方）
        flag = 1;                             % 将标志位设置为1，表示路径与地形相交
        break                                 % 跳出循环，不再判断后续点
    end
end

% 检查生成的路径与威胁物的距离
T = zeros(size(X_seq, 2), size(menace,1));       % 威胁区的威胁程度矩阵初始化
for i = 1:size(X_seq, 2)
    x = X_seq(i);
    y = Y_seq(i);

    % 每个路径节点与威胁物中点的距离检查
    for ii=1:size(menace,1)
        P = menace(ii,:);                     % 获取每个威胁区中点的位置
        d = pdist([[x y];P],'euclidean');     % 计算当前路径点与威胁物之间的欧几里德距离

        % 如果距离小于威胁区域距离，改变标志位
        if d < D + menace_obj_R(ii)           % D 是无人机直径
            flag = 1;
            break
        % 如果距离大于危险区域距离
        elseif d > S + D + menace_obj_R(ii)   % S 是危险区域
            T(i,ii) = 0;                      % 路径点与威胁物之间没有危险
        % 在区域中
        elseif d > D + menace_obj_R(ii) && d < S + D + menace_obj_R(ii)
            T(i,ii)=S+D+menace_obj_R(ii)-d;   % 危险区域的距离减去实际距离，表示在危险区域内的威胁程度
        end
    end
end

% 检查飞行路径是否超出高度上限，如果是，将标志位置为 1，表示违反了高度约束
if sum(find(z_seq > mapRange(3))) >= 1        % find 函数查找非零元素的索引，函数中的不等式成立即为真（1），不成立为假（0）
    flag = 1;
end

% 检查当前点的转弯角度，超过设定的最大角度 Bmax，将标志位 flag 置1，表示违反了转弯角度约束
for i = 2:size(X_seq, 2)-1
    % 获取当前点的坐标
    x = X_seq(i);
    y = Y_seq(i);
    % 获取当前点的前后两个点的坐标
    x1 = X_seq(i+1);
    y1 = Y_seq(i+1);
    x2 = X_seq(i-1);
    y2 = Y_seq(i-1);
    % 计算转弯角度的中间变量
    f1 = (x-x2)*(x1-x)+(y-y2)*(y1-y);         % 表示两个向量的点积
    v1_mo = sqrt((x-x2)^2+(y-y2)^2);          % 表示向量v1的模
    v2_mo = sqrt((x1-x)^2+(y1-y)^2);          % 表示向量v2的模
    f2 = v1_mo * v2_mo;                       % 模长乘积
    B = acos(f1/f2);                          % 向量的点积和模长计算两个向量之间夹角的余弦值，然后使用反余弦函数计算夹角的弧度值

    % 判断是否满足条件
    if B > Bmax
        flag=1;
        break
    end
end

% 检查当前点的转弯角度，超过设定的最大角度 Umax，将标志位 flag 置1，表示违反了转弯角度约束
for i = 2:size(X_seq,2)
    % 获取当前点的坐标
    x = X_seq(i);
    y = Y_seq(i);
    z = Z_seq(i);
    % 获取前一个点的坐标
    x1 = X_seq(i-1);
    y1 = Y_seq(i-1);
    z1 = Z_seq(i-1);
    % 通过反切三角函数计算出俯仰角（atan（对边/邻边））
    u=atan(abs(z-z1) / sqrt((x-x1)^2+(y-y1)^2));

    % 判断是否满足条件，并置标志位
    if u > Umax
        flag = 1;
        break;
    end
end

% 检查是否满足所有约束，如果满足，则计算适应度，否则将适应度设为5000
if flag == 0
    % 路径长度代价
    dx = diff(X_seq);                         % 计算路径在 x 上相邻点的差值
    dy = diff(Y_seq);                         % 计算路径在 y 上相邻点的差值
    dz = diff(Z_seq);                         % 计算路径在 z 上相邻点的差值
    fitness1 = sum(sqrt(dx.^2 + dy.^2 + dz.^2))*(0.9-mm);
    % 威胁代价
    fitness2 = sum(sum(T))*mm;
    % 高度代价：频繁的调整飞行高度不仅会增加能源消耗，同时也不利于飞行的安全，因此希望无人机在飞行过程中尽可能保持平稳
    fitness3 = sum(M*g*sqrt(dz.^2))*0.1;      % M 为无人机总质量， g 为重力加速度，dz 为前后两个航迹点的高度差
    
    % 总和代价（进行了加权）
    fitness = fitness1+fitness2+fitness3;
else
    % 不满足约束，适应度直接5000
    fitness=5000;
end

end