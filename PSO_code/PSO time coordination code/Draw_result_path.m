function Draw_result_path(startPos,goalPos,X,Y,Z,zbest,dim, menaceParams)
% --------------------------------------------------------------
% Draw_result_path - 绘制结果路径的函数
% --------------------------------------------------------------
% startPos - 起点坐标 [x, y, z]的数组---> 5个起点
% goalPos  - 终点坐标 [x, y, z]
% X, Y, Z  - 山峰曲面的坐标
% zbest    - 优化结果
% dim      - 优化变量的维度
% menaceParams:威胁区信息的结构体
% --------------------------------------------------------------

% 第一张子图：三维路线图
figure
subplot(1, 2, 1);

% ---------------------------- 画起点和终点----------------------------------
p1 = scatter3(startPos(1,1), startPos(1,2), startPos(1,3),100,'bs','MarkerFaceColor','y');
hold on
for i=2:5
    p1 = scatter3(startPos(i,1), startPos(i,2), startPos(i,3),100,'bs','MarkerFaceColor','y');
end
p2 = scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');


% ---------------------------- 画山峰曲面----------------------------------
surf(X,Y,Z)         % 画曲面图
hold on;
shading interp;     % 使用插值方法进行着色，使得曲面上的颜色更加平滑
colormap summer;    % 内置颜色映射表，表示夏季风格的颜色，从深蓝渐变到浅黄

% ---------------------------- 威胁物绘制----------------------------------
% 绘制威胁物
alpha1=0.4;         % 不透明度
for i = 1:numel(menaceParams)
    menaceParam = menaceParams(i);
    [Xm, Ym, Zm] = myplotcylinder(menaceParam.start, menaceParam.end, menaceParam.radius);
    p3 = surf(Xm, Ym, Zm, 'facecolor', [2 48 71]/255, 'edgecolor', 'none', 'FaceAlpha', alpha1);
    hold on;
end

% 添加坐标轴信息
xlabel('x(m)','FontName','Times New Roman')
ylabel('y(m)','FontName','Times New Roman')
zlabel('z(m)','FontName','Times New Roman')
% 限制坐标轴的范围
xlim([0,500]);
ylim([0,500]);

% ---------------------------- 路径绘制----------------------------------
% 定义航迹节点变量
x_seq = zeros(5, 6);
y_seq = zeros(5, 6);
z_seq = zeros(5, 6);
% 获取粒子群优化算法迭代的航迹节点
for i=1:5
    x_seq(i,:) = [startPos(i,1), zbest(i,1:dim), goalPos(1)];
    y_seq(i,:) = [startPos(i,2), zbest(i,dim+1:2*dim), goalPos(2)];
    z_seq(i,:) = [startPos(i,3), zbest(i,2*dim+1:3*dim), goalPos(3)];
end

% 使用 spline 函数对路径进行三次样条插值，生成更多的点，使路径更加平滑
k = length(x_seq(1,:));                       % 获取原始路径节点的数量
i_seq = linspace(0,100,k);                    % 生成长度为 k 的等差数列，表示原始节点的归一化位置
xx_seq = linspace(0,100,100);                 % 生成长度为 100 的等差数列，表示插值后路径的归一化位置
yy_seq = linspace(0,100,100);                 % 生成长度为 100 的等差数列，表示插值后路径的归一化位置
zz_seq = linspace(0,100,100);                 % 生成长度为 100 的等差数列，表示插值后路径的归一化位置
for i=1:5
    X_seq(i,:) = spline(i_seq,x_seq(i,:),xx_seq);                   % 利用三次样条进行插值，得到插值预测的100个路径节点
    Y_seq(i,:) = spline(i_seq,y_seq(i,:),yy_seq);                   % 利用三次样条进行插值，得到插值预测的100个路径节点
    Z_seq(i,:) = spline(i_seq,z_seq(i,:),zz_seq);                   % 利用三次样条进行插值，得到插值预测的100个路径节点
    path(i,:,:) = [X_seq(i,:)', Y_seq(i,:)', Z_seq(i,:)'];          % 得到三维路径的矩阵
end

%  通过 plot3 函数画出无人机路线
for i=1:5
    p4 = plot3(path(i,:,1), path(i,:,2),path(i,:,3),'Color',[217 79 51]/255,'LineWidth',2);
end
p4.LineWidth = 3;
hold off
grid on

% legend 函数用于添加图例
legend([p1,p2,p3,p4],'起点','终点','威胁区','无人机路线')
% 添加标题
title('三维路线图')

% -----------------------------------------------------------------------------------------------
% 第二张子图：二维路线图
subplot(1, 2, 2);

% ---------------------------- 画起点和终点----------------------------------
scatter3(startPos(1,1), startPos(1,2), startPos(1,3),100,'bs','MarkerFaceColor','y');
hold on
for i=2:5
    scatter3(startPos(i,1), startPos(i,2), startPos(i,3),100,'bs','MarkerFaceColor','y');
end
scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');

% ---------------------------- 画山峰曲面----------------------------------
surf(X,Y,Z)         % 画曲面图
hold on;
shading interp;     % 使用插值方法进行着色，使得曲面上的颜色更加平滑
colormap summer;    % 内置颜色映射表，表示夏季风格的颜色，从深蓝渐变到浅黄

% ---------------------------- 威胁物绘制----------------------------------
% 绘制威胁物
alpha1=0.4;         % 不透明度
for i = 1:numel(menaceParams)
    menaceParam = menaceParams(i);
    [Xm, Ym, Zm] = myplotcylinder(menaceParam.start, menaceParam.end, menaceParam.radius);
    surf(Xm, Ym, Zm, 'facecolor', [2 48 71]/255, 'edgecolor', 'none', 'FaceAlpha', alpha1);
    hold on;
end

% 添加坐标轴信息
xlabel('x(m)','FontName','Times New Roman')
ylabel('y(m)','FontName','Times New Roman')
% 限制坐标轴的范围
xlim([0,500]);
ylim([0,500]);

% ---------------------------- 路径绘制----------------------------------
%  通过 plot3 函数画出无人机路线
for i=1:5
    plot3(path(i,:,1), path(i,:,2),path(i,:,3),'Color',[217 79 51]/255,'LineWidth',3);
end

% 添加图像标题
title('路线俯视图')
% 看俯视图
view(2)

end
