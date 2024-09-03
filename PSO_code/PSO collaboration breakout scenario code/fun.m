function fitness = fun(pop, params)
% -------------------------------------------------------------
% 适应度函数，用于计算粒子群算法中每个粒子的适应度值
% 适应度规则：判断与地形是否相交、与威胁物距离、俯仰角和转角是否满足，不满足的用罚函数处理
% -------------------------------------------------------------
    % 提取参数
    startPos = params.startPos;         % 起始点的坐标 [x, y, z]
    goalPos = params.goalPos;           % 目标点的坐标 [x, y, z]
    menace = params.menace;             % 威胁物的坐标矩阵，每行表示威胁物的位置 [x, y, z]
    S = params.S;                       % 安全距离
    D = params.D;                       % 毁灭区域半径
    M = params.M;                       % 高度代价权重
    g = params.g;                       % 重力加速度
    dim = params.dim;                   % 经过节点的个数
    menace_obj_R = params.menace_obj_R; % 威胁物的影响半径
    mapRange = params.mapRange;         % 地图的范围 [x_max, y_max, z_max]
    Bmax = params.Bmax;                 % 最大转弯角度
    Umax = params.Umax;                 % 最大俯仰角度
    mm = params.mm;                     % 调节适应度的权重系数

    % 获取插值点数据，即路径的离散节点
    x_seq=[startPos(1), pop(1:dim), goalPos(1)];
    y_seq=[startPos(2), pop(dim+1:2*dim), goalPos(2)];
    z_seq=[startPos(3), pop(2*dim+1:3*dim), goalPos(3)];
    
    k = length(x_seq);
    i_seq = linspace(0,1000,k);
    interp = linspace(0,1000,1000);

    % 使用spline函数对路径进行三次样条插值
    X_seq = spline(i_seq, x_seq, interp);
    Y_seq = spline(i_seq, y_seq, interp);
    Z_seq = spline(i_seq, z_seq, interp);

    % 约束判断
    flag = 0;                                        % 初始化标志位，判断是否满足约束

    T = zeros(size(X_seq, 2), size(menace,1));       % 威胁区的威胁程度矩阵初始化
    for i = 1:size(X_seq, 2)
        x = X_seq(i);
        y = Y_seq(i);
        z = Z_seq(i);
    
        % 威胁区约束检查
        for ii=1:size(menace,1)
            P = menace(ii,:);                     % 获取每个威胁区中点的位置
            d = pdist([[x y];P],'euclidean');     % 计算当前路径点与威胁物之间的欧几里德距离
    
            if d < D + menace_obj_R(ii)
                flag = 1;
                break;
            elseif d > S + D + menace_obj_R(ii)
                T(i,ii) = 0;
            elseif d > D + menace_obj_R(ii) && d < S + D + menace_obj_R(ii)
                T(i,ii)=S+D+menace_obj_R(ii)-d;
            end
        end

        % 高度约束检查
        if z > mapRange(3)
            flag = 1;
        end
    end

    % 航向角度约束
    for i = 2:size(X_seq, 2)-1
        [x, y] = deal(X_seq(i), Y_seq(i));
        [x1, y1] = deal(X_seq(i-1), Y_seq(i-1));
        [x2, y2] = deal(X_seq(i+1), Y_seq(i+1));

        % 计算转弯角度的中间变量
        v1 = [x-x1, y-y1];
        v2 = [x2-x, y2-y];
        B = acos(dot(v1, v2) / (norm(v1) * norm(v2)));
    
        % 判断是否满足约束
        if B > Bmax
            flag=1;
            break
        end
    end

    % 俯仰角约束
    for i = 2:size(X_seq,2)
        [x1, y1, z1] = deal(X_seq(i-1), Y_seq(i-1), Z_seq(i-1));
        [x, y, z] = deal(X_seq(i), Y_seq(i), Z_seq(i));
        u = atan(abs(z - z1) / sqrt((x - x1)^2 + (y - y1)^2));
    
        % 判断是否满足约束
        if u > Umax
            flag = 1;
            break;
        end
    end

    % 适应度计算
    if flag == 0
        % 路径长度代价
        dx = diff(X_seq);
        dy = diff(Y_seq);
        dz = diff(Z_seq);

        fitness1 = sum(sqrt(dx.^2 + dy.^2 + dz.^2))*(0.9-mm);   % 路径约束
        fitness2 = sum(sum(T))*mm;                              % 威胁代价
        fitness3 = sum(M*g*sqrt(dz.^2))*0.1;                    % 高度代价
        fitness = fitness1+fitness2+fitness3;                   % 总和代价（进行了加权）
    else
        fitness=1e12;
    end
end
