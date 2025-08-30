function last_rate = calculate_rate(filename)
% CALCULATE_RATE  从文件中的数据计算变化率。
%   last_rate = CALCULATE_RATE(filename)
%
%   输入:
%       filename: 用于读取数据的文件名。
%
%   输出:
%       last_rate: 计算得出的最终速率。

fp=fopen(filename,'r');
if fp == -1
    error('无法打开文件: %s', filename);
end
fscanf(fp,'%s',1);          % 读取一个字符串

b=zeros(96,1);            % 保存: Bus  V  Angle
for i=1:96
    b(i,1)=fscanf(fp,'%f',1);
end
fclose(fp);


rate=zeros(95,1);

% for i=1:95
%     for j=1:30
%     rate(i)=rate(i)+(b(i+1,j)-b(i,j))/b(i+1,j);
%     end
% end
% rate=rate/30;


for i=1:95
    % 添加除零检查
    if b(i+1,1) ~= 0
        rate(i)=(b(i+1,1)-b(i,1))/b(i+1,1);
    else
        rate(i) = 0; % 或者其他合适的默认值
        warning('除零错误在索引 %d: b(%d,1) = 0', i, i+1);
    end
end

last_rate=zeros(96,1);
last_rate(1)=1;
for i=2:96
    last_rate(i)=last_rate(i-1)+last_rate(i-1)*rate(i-1);
end

%     fp=fopen('Rate_data_4_1.txt','w');
%     fprintf(fp, '   rate_data        ');
%     for i=1:95
%         fprintf(fp, '\n%5d%9.3f',i,rate(i));
%     end
%     fclose(fp);
end