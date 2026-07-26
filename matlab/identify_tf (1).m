% 从升温曲线辨识被控对象传递函数
clear; close all;

%% 读取数据
fid = fopen('升温.txt', 'r');
data = textscan(fid, '[Rx][%d:%d:%f] %f');
fclose(fid);

hour = double(data{1});
min = double(data{2});
sec = data{3};
temp = data{4};

t = hour * 3600 + min * 60 + sec;
t = t - t(1);
y = temp(:);
u0 = y(1);
dy = y(end) - u0;

fprintf('初始温度: %.2f°C, 稳态温度: %.2f°C, 温升: %.2f°C\n', u0, y(end), dy);
fprintf('采样点数: %d, 总时长: %.1f s\n\n', length(t), t(end));

%% 方法1: FOPDT 图解法 (基于 28.3% 和 63.2% 响应点)
y_632 = u0 + 0.632 * dy;
y_283 = u0 + 0.283 * dy;
t_632 = find_crossing(t, y, y_632);
t_283 = find_crossing(t, y, y_283);

T_fopdt = (t_632 - t_283) / log(3);
tau_fopdt = max(t_632 - T_fopdt, 0);
y_fopdt = u0 + dy * (1 - exp(-(t - tau_fopdt) / T_fopdt)) .* (t >= tau_fopdt);
e_fopdt = rms(y - y_fopdt);

fprintf('=== 方法1: FOPDT 图解法 ===\n');
fprintf('  T = %.2f s, τ = %.2f s, RMS = %.4f\n', T_fopdt, tau_fopdt, e_fopdt);

%% 方法2: 一阶无滞后 (63.2% 法)
T_1st = t_632;
y_1st = u0 + dy * (1 - exp(-t / T_1st));
e_1st = rms(y - y_1st);

fprintf('\n=== 方法2: 一阶无滞后 ===\n');
fprintf('  T = %.2f s, RMS = %.4f\n', T_1st, e_1st);

%% 方法3: 暴力扫描 (T, τ)，找一阶加滞后最优拟合
fprintf('\n=== 方法3: 暴力扫描 FOPDT 参数 ===\n');

tau_scan = 0:5:300;
T_scan = 50:5:500;
e_min = inf;
tau_opt = 0;
T_opt = 0;

for tau_try = tau_scan
    for T_try = T_scan
        y_model = u0 + dy * (1 - exp(-(t - tau_try) / T_try)) .* (t >= tau_try);
        e = sqrt(mean((y - y_model).^2));
        if e < e_min
            e_min = e;
            tau_opt = tau_try;
            T_opt = T_try;
        end
    end
end

y_opt = u0 + dy * (1 - exp(-(t - tau_opt) / T_opt)) .* (t >= tau_opt);
fprintf('  最优: T = %d s, τ = %d s, RMS = %.4f  ★\n', T_opt, tau_opt, e_min);

% 在最优值附近精细扫描
tau_fine = max(0, tau_opt-5):1:tau_opt+5;
T_fine = max(10, T_opt-10):1:T_opt+10;
e_min2 = inf;
for tau_try = tau_fine
    for T_try = T_fine
        y_model = u0 + dy * (1 - exp(-(t - tau_try) / T_try)) .* (t >= tau_try);
        e = sqrt(mean((y - y_model).^2));
        if e < e_min2
            e_min2 = e;
            tau_opt2 = tau_try;
            T_opt2 = T_try;
        end
    end
end

y_opt2 = u0 + dy * (1 - exp(-(t - tau_opt2) / T_opt2)) .* (t >= tau_opt2);
fprintf('  精细扫描: T = %d s, τ = %d s, RMS = %.4f  ★\n', T_opt2, tau_opt2, e_min2);

%% 绘制对比图
figure('Position', [100, 100, 1100, 600]);

% 实际数据 - 实线
p0 = plot(t, y, 'k-', 'LineWidth', 1.5); hold on;

% FOPDT 图解法 - 虚线
p1 = plot(t, y_fopdt, 'r--', 'LineWidth', 1.5);

% 一阶无滞后 - 点线
p2 = plot(t, y_1st, 'b:', 'LineWidth', 2);

% FOPDT 最优 - 点划线
p3 = plot(t, y_opt2, 'g-.', 'LineWidth', 2);

plot([0 t(end)], [35.2 35.2], ':', 'Color', [0.6 0.6 0.6]);

xlabel('时间 (s)');
ylabel('温度 (°C)');
title('升温曲线与辨识模型对比');
grid on; box on;
xlim([0, t(end)]);
ylim([u0-1, max(y)+2]);
legend([p0 p1 p2 p3], ...
    {'实际温度', ...
     sprintf('FOPDT图解: T=%.1fs, τ=%.1fs', T_fopdt, tau_fopdt), ...
     sprintf('一阶无滞后: T=%.1fs', T_1st), ...
     sprintf('FOPDT最优: T=%ds, τ=%ds', T_opt2, tau_opt2)}, ...
    'Location', 'southeast');

%% 输出总结
fprintf('\n========== 模型对比总结 ==========\n');
fprintf('%-30s %10s %10s %12s\n', '模型', 'T(s)', 'τ(s)', 'RMS误差');
fprintf('------------------------------------------------------\n');
fprintf('%-30s %10.1f %10.1f %12.4f\n', 'FOPDT 图解法', T_fopdt, tau_fopdt, e_fopdt);
fprintf('%-30s %10.1f %10s %12.4f\n', '一阶无滞后(图解法)', T_1st, '-', e_1st);
fprintf('%-30s %10d %10d %12.4f  ★\n', 'FOPDT 暴力扫描最优', T_opt2, tau_opt2, e_min2);
fprintf('======================================================\n');
fprintf('\n推荐传递函数 (一阶加滞后):\n');
fprintf('  G(s) = %.4f / (%.4f * s + 1) * exp(-%d * s)\n', 1, T_opt2, tau_opt2);
fprintf('  (增益 K 已归一化为 1，实际 K = %.4f / 阶跃幅值)\n', dy);

%% 辅助函数
function t_cross = find_crossing(t, y, y_target)
    idx = find(y >= y_target, 1, 'first');
    if isempty(idx) || idx == 1
        t_cross = t(end);
        return;
    end
    t0 = t(idx-1); t1 = t(idx);
    y0 = y(idx-1); y1 = y(idx);
    t_cross = t0 + (y_target - y0)*(t1 - t0)/(y1 - y0);
end
