% 读取升温数据并绘制时间-温度曲线
fid = fopen('升温.txt', 'r');
data = textscan(fid, '[Rx][%d:%d:%f] %f');
fclose(fid);

% 解析时间（相对秒数，从第一个采样点开始）
hour = double(data{1});
min = double(data{2});
sec = data{3};
temp = data{4};

t = hour * 3600 + min * 60 + sec;
t = t - t(1);  % 归零

% 绘制曲线
figure('Position', [100, 100, 900, 500]);
plot(t, temp, 'b-', 'LineWidth', 1);
hold on;
plot([0, t(end)], [35.2, 35.2], 'r--', 'LineWidth', 1);
hold off;
xlabel('时间 (s)');
ylabel('温度 (°C)');
title('升温曲线');
legend('实测温度', '目标温度 35.2°C', 'Location', 'best');
grid on;
xlim([0, t(end)]);
