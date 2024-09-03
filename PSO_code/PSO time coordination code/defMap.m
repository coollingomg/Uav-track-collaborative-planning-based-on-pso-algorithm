function [X,Y,Z] = defMap(mapRange,N)
% -------------------------------------------------------------
% ˵������������ɽ���ͼ�ĺ���defMap
% -------------------------------------------------------------
% ������
% mapRange����һ��������ͼ�������߷�Χ������
% N����ɽ��ĸ���
% ���������������� X,Y,Z���ֱ��ʾ������ x��y��z ����
% -------------------------------------------------------------

% ɽ�����������ĳ�ʼ��
peaksInfo = struct;                 % ��ʼ��ɽ��������Ϣ�ṹ��
peaksInfo.center = [];              % ɽ������
peaksInfo.range = [];               % ɽ������
peaksInfo.height = [];              % ɽ��߶�
peaksInfo = repmat(peaksInfo,N,1);  % ����N��ɽ��ṹ�幹�ɾ���

% Ԥ����ɽ�����������
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

% ����ɽ�壬Ϊÿ��ɽ����������ĸ߶Ⱥͷ�Χ�����������ĵ�
for i = 1:N
    peaksInfo(i).center = center(i,:);
    peaksInfo(i).height = mapRange(3) * (rand*0.8+0.2);
    peaksInfo(i).range = mapRange * 0.1 * (rand*0.7+0.3);
end

%-------------------����ɽ������ֵ-------------------%
% ��ʼ��һ������ peakData �洢ɽ�������ֵ
peakData = zeros(mapRange(1), mapRange(2));
% ʹ�ø�˹��������ÿ�����ɽ������ֵ
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

%-------------------�������ɵĵ�ͼ���ݵ���Ϣ-------------------%
[x , y] = meshgrid(1:mapRange(1), 1:mapRange(2));
X = x;
Y = y;
Z = peakData;

end
