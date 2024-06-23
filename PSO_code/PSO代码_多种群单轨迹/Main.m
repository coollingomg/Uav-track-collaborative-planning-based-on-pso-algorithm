%% 运行两个主程序
% 运行两个算法程序
IDM_PSO 
PSO

% 清空缓存
clear,clc,close all

%% 对比
% 加载运行的数据
load('PSO.mat');
load('IDM_PSO.mat');

% 画图展现
figure

% 1 迭代图
plot(1:length(yy1),yy1,'Color',[233 196 107]/255,'LineWidth',2);
hold on;
plot(1:length(yy3),yy3,'Color',[38 70 83]/255,'LineWidth',2)

grid on
legend('PSO','IDM-PSO','FontSize',12,'FontName','Times New Roman')
xlabel('进化代数','FontSize',12)
ylabel('最优个体适应度','FontSize',12)
title('进化曲线对比','FontSize',14)

% 2 三维图
figure
subplot(121)
% 画起点和终点
p1 = scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y');
hold on
p2 = scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');

% 画山峰曲面
surf(X,Y,Z)      % 画曲面图
shading flat     % 各小曲面之间不要网格
colormap summer ;

% 威胁物
alpha=0.4; % 不透明度
a1=[270 200 0];
b1=[270 200 100];
R1 = 20;
[X1,Y1,Z1] = myplotcylinder(a1,b1,R1);
p3 = surf(X1,Y1,Z1,'facecolor',[2 48 71]/255,'edgecolor','none','FaceAlpha',alpha);hold on   %圆柱表面
a2=[170 350 0];
b2=[170 350 100];
R2 = 30;
[X2,Y2,Z2] = myplotcylinder(a2,b2,R2);
surf(X2,Y2,Z2,'facecolor',[2 48 71]/255,'edgecolor','none','FaceAlpha',alpha)    %圆柱表面
a3=[300 300 0];
b3=[300 300 100];
R3 = 25;
[X3,Y3,Z3] = myplotcylinder(a3,b3,R3);
surf(X3,Y3,Z3,'facecolor',[2 48 71]/255,'edgecolor','none','FaceAlpha',alpha)    %圆柱表面
a4=[350 400 0];
b4=[350 400 100];
R4 = 30;
[X4,Y4,Z4] = myplotcylinder(a4,b4,R4);
surf(X4,Y4,Z4,'facecolor',[2 48 71]/255,'edgecolor','none','FaceAlpha',alpha)    %圆柱表面
xlabel('x(m)','FontName','Times New Roman')
ylabel('y(m)','FontName','Times New Roman')
zlabel('z(m)','FontName','Times New Roman')
xlim([0,500]);ylim([0,500]);

% 画路径

% 利用三次样条拟合散点
pop = zbest1;
x_seq=[startPos(1), pop(1:dim), goalPos(1)];
y_seq=[startPos(2), pop(dim+1:2*dim), goalPos(2)];
z_seq=[startPos(3), pop(2*dim+1:3*dim), goalPos(3)];

k = length(x_seq);
i_seq = linspace(0,1,k);
xx_seq = linspace(0,1,100);
yy_seq = linspace(0,1,100);
zz_seq = linspace(0,1,100);
X_seq = spline(i_seq,x_seq,xx_seq);
Y_seq = spline(i_seq,y_seq,yy_seq);
Z_seq = spline(i_seq,z_seq,zz_seq);
path = [X_seq', Y_seq', Z_seq'];

pos = [[pop(1:dim)]' [pop(dim+1:2*dim)]' [pop(2*dim+1:3*dim)]'];
p4 = plot3(path(:,1), path(:,2),path(:,3),'Color',[233 196 107]/255,'LineWidth',2);
p4.LineWidth=3;

% 利用三次样条拟合散点
pop = zbest3;
x_seq=[startPos(1), pop(1:dim), goalPos(1)];
y_seq=[startPos(2), pop(dim+1:2*dim), goalPos(2)];
z_seq=[startPos(3), pop(2*dim+1:3*dim), goalPos(3)];

k = length(x_seq);
i_seq = linspace(0,1,k);
xx_seq = linspace(0,1,100);
yy_seq = linspace(0,1,100);
zz_seq = linspace(0,1,100);
X_seq = spline(i_seq,x_seq,xx_seq);
Y_seq = spline(i_seq,y_seq,yy_seq);
Z_seq = spline(i_seq,z_seq,zz_seq);
path = [X_seq', Y_seq', Z_seq'];

pos = [[pop(1:dim)]' [pop(dim+1:2*dim)]' [pop(2*dim+1:3*dim)]'];

p6 = plot3(path(:,1), path(:,2),path(:,3),'Color',[38 70 83]/255,'LineWidth',2);

hold off
grid on

legend([p1,p2,p3,p4,p6],'起点','终点','威胁区','PSO路线','IDM-PSO路线')
title('三维路线图')


subplot(122)
% alpha=1;
% 画起点和终点
p1 = scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y');
hold on
p2 = scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y');

% 画山峰曲面
surf(X,Y,Z)      % 画曲面图
shading flat     % 各小曲面之间不要网格
colormap summer ;
p3 = surf(X1,Y1,Z1,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha);hold on 
surf(X2,Y2,Z2,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha) 
surf(X3,Y3,Z3,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha)
surf(X4,Y4,Z4,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha) 
scatter3(b1(1),b1(2),b1(3),20,'MarkerFaceColor',[158 49 80]/255,'MarkerEdgeColor','none')
scatter3(b2(1),b2(2),b2(3),20,'MarkerFaceColor',[158 49 80]/255,'MarkerEdgeColor','none')
scatter3(b3(1),b3(2),b3(3),20,'MarkerFaceColor',[158 49 80]/255,'MarkerEdgeColor','none')
scatter3(b4(1),b4(2),b4(3),20,'MarkerFaceColor',[158 49 80]/255,'MarkerEdgeColor','none')
% 利用三次样条拟合散点
pop = zbest1;
x_seq=[startPos(1), pop(1:dim), goalPos(1)];
y_seq=[startPos(2), pop(dim+1:2*dim), goalPos(2)];
z_seq=[startPos(3), pop(2*dim+1:3*dim), goalPos(3)];

k = length(x_seq);
i_seq = linspace(0,1,k);
xx_seq = linspace(0,1,100);
yy_seq = linspace(0,1,100);
zz_seq = linspace(0,1,100);
X_seq = spline(i_seq,x_seq,xx_seq);
Y_seq = spline(i_seq,y_seq,yy_seq);
Z_seq = spline(i_seq,z_seq,zz_seq);
path = [X_seq', Y_seq', Z_seq'];

pos = [[pop(1:dim)]' [pop(dim+1:2*dim)]' [pop(2*dim+1:3*dim)]'];
% scatter3(pos(:,1), pos(:,2), pos(:,3), 'go');
p4 = plot3(path(:,1), path(:,2),path(:,3),'Color',[233 196 107]/255,'LineWidth',2);
p4.LineWidth=3;

% 利用三次样条拟合散点
pop = zbest3;
x_seq=[startPos(1), pop(1:dim), goalPos(1)];
y_seq=[startPos(2), pop(dim+1:2*dim), goalPos(2)];
z_seq=[startPos(3), pop(2*dim+1:3*dim), goalPos(3)];

k = length(x_seq);
i_seq = linspace(0,1,k);
xx_seq = linspace(0,1,100);
yy_seq = linspace(0,1,100);
zz_seq = linspace(0,1,100);
X_seq = spline(i_seq,x_seq,xx_seq);
Y_seq = spline(i_seq,y_seq,yy_seq);
Z_seq = spline(i_seq,z_seq,zz_seq);
path = [X_seq', Y_seq', Z_seq'];

pos = [[pop(1:dim)]' [pop(dim+1:2*dim)]' [pop(2*dim+1:3*dim)]'];
p6 = plot3(path(:,1), path(:,2),path(:,3),'Color',[38 70 83]/255,'LineWidth',2);
p6.LineWidth=3;

% 加上威胁物的警戒区域
a1=[270 200 0];
b1=[270 200 100];
R1 = 20+D;
[X1,Y1,Z1] = myplotcylinder(a1,b1,R1);
surf(X1,Y1,Z1,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha);hold on   %圆柱表面
a1=[270 200 0];
b1=[270 200 100];
R1 = 20+S+D;
[X1,Y1,Z1] = myplotcylinder(a1,b1,R1);
surf(X1,Y1,Z1,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha);hold on   %圆柱表面
a2=[170 350 0];
b2=[170 350 100];
R2 = 30+D;
[X2,Y2,Z2] = myplotcylinder(a2,b2,R2);
surf(X2,Y2,Z2,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha)    %圆柱表面
a2=[170 350 0];
b2=[170 350 100];
R2 = 30+S+D;
[X2,Y2,Z2] = myplotcylinder(a2,b2,R2);
surf(X2,Y2,Z2,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha)    %圆柱表面
a3=[300 300 0];
b3=[300 300 100];
R3 = 25+D;
[X3,Y3,Z3] = myplotcylinder(a3,b3,R3);
surf(X3,Y3,Z3,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha)    %圆柱表面
a3=[300 300 0];
b3=[300 300 100];
R3 = 25+S+D;
[X3,Y3,Z3] = myplotcylinder(a3,b3,R3);
surf(X3,Y3,Z3,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha)    %圆柱表面
a4=[350 400 0];
b4=[350 400 100];
R4 = 30+S+D;
[X4,Y4,Z4] = myplotcylinder(a4,b4,R4);
surf(X4,Y4,Z4,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha)    %圆柱表面
a4=[350 400 0];
b4=[350 400 100];
R4 = 30+D;
[X4,Y4,Z4] = myplotcylinder(a4,b4,R4);
surf(X4,Y4,Z4,'facecolor',[2 48 71]/255,'edgecolor',[2 48 71]/255,'FaceAlpha',alpha)    %圆柱表面
xlabel('x(m)','FontName','Times New Roman')
ylabel('y(m)','FontName','Times New Roman')


title('路线俯视图')

view(2)
