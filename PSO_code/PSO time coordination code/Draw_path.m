function Draw_path(startPos,goalPos,X,Y,Z,zbest,dim,menaceParams)
% --------------------------------------------------------------
% 绘制路径规划的图形，包括起点、终点、山峰地形、威胁物、无人机路径等
% --------------------------------------------------------------
% 输入参数：
% startPos: 起点坐标 [x, y, z]
% goalPos: 终点坐标 [x, y, z]
% X, Y, Z: 地形数据的网格坐标
% zbest: 最佳路径的坐标数组
% dim: 路径节点数
% menaceParams：威胁区信息的结构体
% --------------------------------------------------------------


% ---------------------------- 画起点和终点----------------------------------
p1 = scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y');
hold on
p2 = scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');


% ---------------------------- 画山峰曲面----------------------------------
surf(X,Y,Z)         % 画曲面图
hold on;
shading interp;     % 使用插值方法进行着色，使得曲面上的颜色更加平滑
colormap summer;    % 内置颜色映射表，表示夏季风格的颜色，从深蓝渐变到浅黄
alpha1=0.4;         % 不透明度

% ---------------------------- 威胁物绘制----------------------------------
% 绘制威胁物
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
% 获取粒子群优化算法迭代的航迹节点
x_seq=[startPos(1), zbest(1:dim), goalPos(1)];
y_seq=[startPos(2), zbest(dim+1:2*dim), goalPos(2)];
z_seq=[startPos(3), zbest(2*dim+1:3*dim), goalPos(3)];

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

%  通过 plot3 函数画出无人机路线
p4 = plot3(path(:,1), path(:,2),path(:,3),'Color',[217 79 51]/255,'LineWidth',2);
hold off
grid on

% legend 函数用于添加图例
legend([p1,p2,p3,p4],'起点','终点','威胁区','无人机路线')

end
