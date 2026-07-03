%% 生化培养箱控制系统 - PID参数整定与仿真
% 被控对象: G(s) = K * e^(-tau*s) / (T1*s + 1)
%           K = 1.2, T1 = 300s, tau = 30s
% 控制要求: 精度 ±1°C, 超调量 < 20%
% PID 初值: Kp=15.0, Ki=0.8, Kd=2.5 (需根据仿真结果调整)

clear; clc; close all;

%% ========== 被控对象模型参数 ==========
K = 1.2;        % 系统增益
T1 = 300;       % 惯性时间常数 (s) — 培养箱热惯性大
tau = 30;       % 纯滞后时间 (s) — 温度变化需时间传导

%% ========== PID 参数 (初始整定值) ==========
Kp = 15.0;      % 比例增益: 越大响应越快, 但过大会引起振荡
Ki = 0.8;       % 积分增益: 消除稳态误差, 过大导致超调
Kd = 2.5;       % 微分增益: 抑制振荡, 改善动态响应

%% ========== 仿真时间参数 ==========
T_sample = 0.2;     % 采样周期 (s) = 200ms (与实物代码一致)
T_sim = 3600;       % 仿真时长 (s) = 1 小时
t = 0:T_sample:T_sim;
N = length(t);

%% ========== 设定值 ==========
setpoint = 37.0;     % 目标温度 (°C)
initial_temp = 25.0; % 初始温度 (°C, 室温)

%% ========== PID 状态变量 ==========
e_prev = 0;
integral = 0;
u = 0;
output = zeros(1, N);
measurement = zeros(1, N);
measurement(1) = initial_temp;
control_signal = zeros(1, N);

%% ========== 被控对象离散化 ==========
% 一阶惯性环节: 后向差分法近似
alpha = T_sample / (T1 + T_sample);
beta  = K * T_sample / (T1 + T_sample);

% 纯滞后: 用 FIFO 环形缓冲模拟
delay_steps = round(tau / T_sample);  % 30s / 0.2s = 150 步
delay_buffer = zeros(1, delay_steps + 1);
delay_idx = 1;

%% ========== 积分分离阈值 ==========
integral_sep_threshold = 3.0;  % |e|>3°C 时 Ki=0

%% ========== 主仿真循环 ==========
fprintf('正在仿真，请稍候...\n');
for k = 1:N-1
    % 从延迟缓冲中读取当前测量值 (考虑滞后)
    measurement(k) = delay_buffer(delay_idx) + initial_temp;

    e = setpoint - measurement(k);

    % ── 积分分离 + 抗积分饱和 ──
    if abs(e) <= integral_sep_threshold
        integral = integral + e * T_sample;
        if integral > 100/Ki, integral = 100/Ki; end
        if integral < -100/Ki, integral = -100/Ki; end
    end

    % ── 位置式 PID ──
    u = Kp * e + Ki * integral + Kd * (e - e_prev) / T_sample;

    % ── 输出限幅 [-100, 100] ──
    if u > 100, u = 100; end
    if u < -100, u = -100; end

    % ── 条件积分法抗饱和 ──
    if u >= 100 && e > 0, integral = integral - e * T_sample; end
    if u <= -100 && e < 0, integral = integral - e * T_sample; end

    control_signal(k) = u;
    e_prev = e;

    % ── 被控对象动态响应 ──
    u_eff = u / 100;  % 归一化控制量 [-1, 1]
    delta = beta * K * u_eff + alpha * delay_buffer(delay_idx);
    delay_buffer(delay_idx) = delay_buffer(delay_idx) + delta;

    delay_idx = delay_idx + 1;
    if delay_idx > delay_steps + 1
        delay_idx = 1;
    end

    output(k) = measurement(k);
end

measurement(N) = delay_buffer(delay_idx) + initial_temp;
output(N) = measurement(N);

%% 性能指标计算
overshoot = 0;
rise_time = 0;
settling_time = N * T_sample;
steady_state = mean(output(round(N*0.8):N));
steady_error = setpoint - steady_state;

% 找到进入稳态 (2% 准则) 的时间
for k = 1:N
    if abs(output(k) - setpoint) > 0.02 * setpoint
        settling_time = k * T_sample;
    end
end

% 计算超调量
[max_val, max_idx] = max(output);
if max_val > setpoint
    overshoot = (max_val - setpoint) / setpoint * 100;
end

% 上升时间 (10% 到 90%)
for k = 1:N
    if output(k) >= 0.1 * setpoint
        t10 = k * T_sample;
        break;
    end
end
for k = 1:N
    if output(k) >= 0.9 * setpoint
        t90 = k * T_sample;
        break;
    end
end
rise_time = t90 - t10;

%% 显示性能指标
fprintf('========== PID 控制性能指标 ==========\n');
fprintf('PID参数: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', Kp, Ki, Kd);
fprintf('设定温度: %.1f °C\n', setpoint);
fprintf('初始温度: %.1f °C\n', initial_temp);
fprintf('稳态温度: %.2f °C\n', steady_state);
fprintf('稳态误差: %.3f °C\n', steady_error);
fprintf('最大超调量: %.2f %%\n', overshoot);
fprintf('上升时间: %.1f s\n', rise_time);
fprintf('调节时间(2%%): %.1f s\n', settling_time);
fprintf('控制精度: ±%.2f °C\n', max(abs(output(N-100:N) - setpoint)));

%% 绘制仿真结果
figure('Name', '生化培养箱 PID 控制系统仿真', 'Position', [100, 100, 1200, 800]);

subplot(3,1,1);
plot(t, output, 'b-', 'LineWidth', 1.5); hold on;
plot(t, setpoint * ones(1,N), 'r--', 'LineWidth', 1.5);
plot(t, (setpoint + 1) * ones(1,N), 'g:', 'LineWidth', 1);
plot(t, (setpoint - 1) * ones(1,N), 'g:', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('温度 (°C)');
title('温度响应曲线');
legend('实际温度', '设定温度', '±1°C 精度带', 'Location', 'best');
grid on;
xlim([0, T_sim]);
ylim([min(output)-2, max(output)+2]);

subplot(3,1,2);
plot(t, control_signal, 'm-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('控制量 (%)');
title('PID 控制输出 (PWM 占空比)');
grid on;
xlim([0, T_sim]);
ylim([-110, 110]);

subplot(3,1,3);
error_signal = setpoint - output;
plot(t, error_signal, 'k-', 'LineWidth', 1.5); hold on;
plot(t, zeros(1,N), 'r--');
xlabel('时间 (s)');
ylabel('误差 (°C)');
title('温度误差曲线');
grid on;
xlim([0, T_sim]);

%% 局部放大图 (显示前 600s 的细节)
figure('Name', '启动阶段细节', 'Position', [100, 100, 1200, 400]);
t_start = 0;
t_end = 600;
idx_range = (t >= t_start) & (t <= t_end);

plot(t(idx_range), output(idx_range), 'b-', 'LineWidth', 1.5); hold on;
plot(t(idx_range), setpoint * ones(1, sum(idx_range)), 'r--', 'LineWidth', 1.5);
plot(t(idx_range), (setpoint + 1) * ones(1, sum(idx_range)), 'g:', 'LineWidth', 1);
plot(t(idx_range), (setpoint - 1) * ones(1, sum(idx_range)), 'g:', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('温度 (°C)');
title('启动阶段温度响应 (0-600s)');
legend('实际温度', '设定温度', '±1°C 精度带', 'Location', 'best');
grid on;

%% PID 参数扫描 (可选)
% 取消注释下面的代码进行参数扫描
%{
fprintf('\n正在参数扫描...\n');
Kp_range = [5, 10, 15, 20, 25];
Ki_range = [0.2, 0.5, 0.8, 1.0, 1.5];
Kd_range = [1.0, 1.5, 2.0, 2.5, 3.0];

figure('Name', '参数扫描', 'Position', [100, 100, 1400, 600]);

% 扫描 Kp
for i = 1:length(Kp_range)
    [out, ~] = run_simulation(Kp_range(i), Ki, Kd, setpoint, initial_temp, T_sim, T_sample, K, T1, tau);
    subplot(1,3,1); hold on;
    plot(t, out, 'DisplayName', sprintf('Kp=%.1f', Kp_range(i)));
end
subplot(1,3,1);
plot(t, setpoint*ones(1,N), 'k--');
xlabel('Time (s)'); ylabel('Temperature (°C)');
title('Kp 扫描'); legend show; grid on;

% 扫描 Ki
for i = 1:length(Ki_range)
    [out, ~] = run_simulation(Kp, Ki_range(i), Kd, setpoint, initial_temp, T_sim, T_sample, K, T1, tau);
    subplot(1,3,2); hold on;
    plot(t, out, 'DisplayName', sprintf('Ki=%.2f', Ki_range(i)));
end
subplot(1,3,2);
plot(t, setpoint*ones(1,N), 'k--');
xlabel('Time (s)'); ylabel('Temperature (°C)');
title('Ki 扫描'); legend show; grid on;

% 扫描 Kd
for i = 1:length(Kd_range)
    [out, ~] = run_simulation(Kp, Ki, Kd_range(i), setpoint, initial_temp, T_sim, T_sample, K, T1, tau);
    subplot(1,3,3); hold on;
    plot(t, out, 'DisplayName', sprintf('Kd=%.1f', Kd_range(i)));
end
subplot(1,3,3);
plot(t, setpoint*ones(1,N), 'k--');
xlabel('Time (s)'); ylabel('Temperature (°C)');
title('Kd 扫描'); legend show; grid on;
%}

%% 辅助仿真函数
function [output, control] = run_simulation(Kp, Ki, Kd, setpoint, initial_temp, T_sim, T_sample, K, T1, tau)
    t = 0:T_sample:T_sim;
    N = length(t);
    e_prev = 0;
    integral = 0;
    u = 0;
    output = zeros(1, N);
    control = zeros(1, N);

    alpha = T_sample / (T1 + T_sample);
    beta = K * T_sample / (T1 + T_sample);

    delay_steps = round(tau / T_sample);
    delay_buffer = zeros(1, delay_steps + 1);
    delay_idx = 1;

    integral_sep = 3.0;

    for k = 1:N-1
        meas = delay_buffer(delay_idx) + initial_temp;
        output(k) = meas;

        e = setpoint - meas;

        if abs(e) <= integral_sep
            integral = integral + e * T_sample;
            if integral > 100/Ki, integral = 100/Ki; end
            if integral < -100/Ki, integral = -100/Ki; end
        end

        u = Kp * e + Ki * integral + Kd * (e - e_prev) / T_sample;

        if u > 100, u = 100; end
        if u < -100, u = -100; end

        if u >= 100 && e > 0, integral = integral - e * T_sample; end
        if u <= -100 && e < 0, integral = integral - e * T_sample; end

        control(k) = u;
        e_prev = e;

        u_eff = u / 100;
        delta = beta * K * u_eff + alpha * delay_buffer(delay_idx);
        delay_buffer(delay_idx) = delay_buffer(delay_idx) + delta;

        delay_idx = delay_idx + 1;
        if delay_idx > delay_steps + 1
            delay_idx = 1;
        end
    end
    output(N) = delay_buffer(delay_idx) + initial_temp;
end

fprintf('\n仿真完成！\n');
