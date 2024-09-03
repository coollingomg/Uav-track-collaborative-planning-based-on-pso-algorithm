function [X,Y,Z] = defMap(mapRange,N)
% -------------------------------------------------------------
% 说明：定义生成山峰地图的函数defMap
% -------------------------------------------------------------
% 参数：
% mapRange：是一个包含地图长、宽、高范围的向量
% N：是山峰的个数
% 函数返回三个矩阵 X,Y,Z，分别表示网格点的 x、y、z 坐标
% -------------------------------------------------------------

% 山峰特征参数的初始化
peaksInfo = struct;                 % 初始化山峰特征信息结构体
peaksInfo.center = [];              % 山峰中心
peaksInfo.range = [];               % 山峰区域
peaksInfo.height = [];              % 山峰高度
peaksInfo = repmat(peaksInfo,N,1);  % 复制N个山峰结构体构成矩阵

% 预定义山峰的中心坐标
center = [445 120;
          50  70;
          150 450;
          170 150;
          400 450;
          350 270;
          70  300;
          250 400;
          300 100;
          170 390];

% 遍历山峰，为每个山峰生成随机的高度和范围，并分配中心点
for i = 1:N
    peaksInfo(i).center = center(i,:);
    peaksInfo(i).height = mapRange(3) * (rand*0.8+0.2);
    peaksInfo(i).range = mapRange * 0.1 * (rand*0.7+0.3);
end

%-------------------计算山峰曲面值-------------------%
% 初始化一个矩阵 peakData 存储山峰的曲面值
peakData = zeros(mapRange(1), mapRange(2));
% 使用高斯函数计算每个点的山峰曲面值
for x = 1:mapRange(1)
    for y = 1:mapRange(2)
        sum = 0;
        for k = 1:N
            h_i = peaksInfo(k).height;
            x_i = peaksInfo(k).center(1);
            y_i = peaksInfo(k).center(2);
            x_si = peaksInfo(k).range(1);
            y_si = peaksInfo(k).range(2);
            sum = sum + h_i * exp(-((x-x_i)/x_si)^2 - ((y-y_i)/y_si)^2);
        end
        peakData(x, y) = sum;
    end
end

%-------------------返回生成的地图数据点信息-------------------%
[x , y] = meshgrid(1:mapRange(1), 1:mapRange(2));
X = x;
Y = y;
Z = peakData;

end
