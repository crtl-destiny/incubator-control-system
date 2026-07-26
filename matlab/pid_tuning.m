%% PID 参数整定
% 被控对象: G(s) = 1/(225s+1) * e^(-62s)
clear; close all;

K  = 1;       % 增益
T  = 225;     % 时间常数 (s)
tau = 62;     % 纯滞后 (s)

fprintf('被控对象: G(s) = %.1f/(%.1fs+1)*exp(-%.1fs)\n', K, T, tau);
fprintf('τ/T = %.3f\n\n', tau/T);

%% 方法1: Ziegler-Nichols 阶跃响应法 (基于 K, T, τ)
fprintf('=== 方法1: Ziegler-Nichols 阶跃响应法 ===\n');
zn_P.Kp   = T/(K*tau);
zn_PI.Kp  = 0.9*T/(K*tau);
zn_PI.Ki  = zn_PI.Kp/(3.3*tau);
zn_PID.Kp = 1.2*T/(K*tau);
zn_PID.Ki = zn_PID.Kp/(2*tau);
zn_PID.Kd = zn_PID.Kp*tau/2;

fprintf('P:   Kp=%.4f\n', zn_P.Kp);
fprintf('PI:  Kp=%.4f, Ki=%.6f\n', zn_PI.Kp, zn_PI.Ki);
fprintf('PID: Kp=%.4f, Ki=%.6f, Kd=%.1f\n', zn_PID.Kp, zn_PID.Ki, zn_PID.Kd);

%% 方法5: 稳定边界法 (临界比例度)
fprintf('\n=== 方法5: 稳定边界法 (特性分析) ===\n');
s = tf('s');
G = K/(T*s+1)*exp(-tau*s);
G_pade = K/(T*s+1) * pade(tau*s, 2);
try
    [Gm, Pm, Wcg, Wcp] = margin(G_pade);
    Kc = Gm;
    Tc = 2*pi/Wcg;
    fprintf('临界增益 Kc = %.4f\n', Kc);
    fprintf('临界振荡周期 Tc = %.2f s\n', Tc);
    fprintf('相角裕度 Pm = %.2f° (Kp=1时)\n', Pm);

    fprintf('\nZ-N 临界比例度法整定:\n');
    zn2_P.Kp   = 0.5*Kc;
    zn2_PI.Kp  = 0.45*Kc;
    zn2_PI.Ki  = zn2_PI.Kp/(0.83*Tc);
    zn2_PID.Kp = 0.6*Kc;
    zn2_PID.Ki = zn2_PID.Kp/(0.5*Tc);
    zn2_PID.Kd = zn2_PID.Kp*Tc/8;
    fprintf('P:   Kp=%.4f\n', zn2_P.Kp);
    fprintf('PI:  Kp=%.4f, Ki=%.6f\n', zn2_PI.Kp, zn2_PI.Ki);
    fprintf('PID: Kp=%.4f, Ki=%.6f, Kd=%.1f\n', zn2_PID.Kp, zn2_PID.Ki, zn2_PID.Kd);
    method5_ok = true;
catch
    fprintf('(无法计算临界增益，跳过)\n');
    method5_ok = false;
end

%% 自定义方案: 近似大林算法 PI
Kp_manual = 1.84;
Ki_manual = 0.0082;
Kd_manual = 0;
fprintf('\n=== 自定义方案 (近似大林算法) ===\n');
fprintf('PI: Kp=%.4f, Ki=%.6f\n', Kp_manual, Ki_manual);

%% 闭环仿真对比
fprintf('\n=== 闭环阶跃响应仿真 ===\n');
Ts_sim = 1;
t_sim = 0:Ts_sim:2000;
r = ones(size(t_sim));

% 待比较的方案
tune_methods = {
    'Z-N PI',     zn_PI.Kp,    zn_PI.Ki,    0;
    'Z-N PID',    zn_PID.Kp,   zn_PID.Ki,   zn_PID.Kd;
    '自定义PI',   Kp_manual,   Ki_manual,   0;
};
if method5_ok
    tune_methods = [tune_methods; {'临界P', zn2_P.Kp, 0, 0}];
    tune_methods = [tune_methods; {'临界PI', zn2_PI.Kp, zn2_PI.Ki, 0}];
    tune_methods = [tune_methods; {'临界PID', zn2_PID.Kp, zn2_PID.Ki, zn2_PID.Kd}];
end

results = {};
h_plots = [];
legend_str = {};
figure('Position', [100, 100, 1200, 700]);

subplot(2,1,1); hold on; grid on;
for m = 1:size(tune_methods,1)
    name = tune_methods{m,1};
    Kp = tune_methods{m,2};
    Ki = tune_methods{m,3};
    Kd = tune_methods{m,4};

    y = zeros(size(t_sim));
    u = zeros(size(t_sim));
    e_int = 0; e_prev = 0;
    u_buf = zeros(1, round(tau/Ts_sim)+2);

    for k = 1:length(t_sim)
        e = r(k) - y(k);
        e_int = e_int + e * Ts_sim;
        u(k) = Kp*e + Ki*e_int + Kd*(e - e_prev)/Ts_sim;
        u(k) = max(0, min(2, u(k)));

        e_prev = e;
        u_buf = [u(k), u_buf(1:end-1)];

        if k < length(t_sim)
            delay_idx = round(tau/Ts_sim) + 1;
            if delay_idx <= length(u_buf)
                y(k+1) = exp(-Ts_sim/T)*y(k) + K*(1-exp(-Ts_sim/T))*u_buf(delay_idx);
            end
        end
    end

    info = stepinfo(y, t_sim, 1);
    overshoot = max(0, info.Overshoot);
    idx_63 = find(y >= 0.632, 1);
    Tr = t_sim(idx_63) - tau; if Tr < 0, Tr = 0; end
    u_max = max(u);
    ess = abs(1 - mean(y(end-100:end)));

    results{m,1} = name;
    results{m,2} = Kp;
    results{m,3} = Ki;
    results{m,4} = Kd;
    results{m,5} = Tr;
    results{m,6} = info.SettlingTime;
    results{m,7} = overshoot;
    results{m,8} = u_max;
    results{m,9} = ess;

    p = plot(t_sim, y, 'LineWidth', 1.5);
    h_plots = [h_plots, p];
    legend_str{end+1} = sprintf('%s (Tr=%.0fs,σ=%.1f%%)', name, Tr, overshoot);
end

% 原始 PID
Kp0=12; Ki0=1; Kd0=0;
y0=zeros(size(t_sim)); u0=zeros(size(t_sim)); e_int0=0; e_prev0=0;
u_buf0=zeros(1, round(tau/Ts_sim)+2);
for k=1:length(t_sim)
    e = r(k)-y0(k);
    e_int0 = e_int0 + e*Ts_sim;
    u0(k) = Kp0*e + Ki0*e_int0 + Kd0*(e - e_prev0)/Ts_sim;
    u0(k)=max(0,min(2,u0(k)));
    e_prev0=e;
    u_buf0=[u0(k), u_buf0(1:end-1)];
    if k<length(t_sim)
        y0(k+1)=exp(-Ts_sim/T)*y0(k)+K*(1-exp(-Ts_sim/T))*u_buf0(round(tau/Ts_sim)+1);
    end
end
p0 = plot(t_sim, y0, 'k-', 'LineWidth', 2);
yline(1, 'r--', 'LineWidth', 0.5);
xlabel('时间 (s)'); ylabel('温度');
title('PID 闭环阶跃响应对比');
legend([p0, h_plots], ['原始 PID (P=12,I=1)', legend_str], 'Location', 'southeast');
xlim([0, 1000]);

% 控制量
subplot(2,1,2); hold on; grid on;
plot(t_sim, u0, 'k-', 'LineWidth', 2);
for m = 1:size(tune_methods,1)
    name = tune_methods{m,1};
    Kp = tune_methods{m,2};
    Ki = tune_methods{m,3};
    Kd = tune_methods{m,4};
    y=zeros(size(t_sim)); u=zeros(size(t_sim));
    e_int=0; e_prev=0; u_buf=zeros(1,round(tau/Ts_sim)+2);
    for k=1:length(t_sim)
        e = r(k)-y(k); e_int=e_int+e*Ts_sim;
        u(k)=Kp*e+Ki*e_int+Kd*(e-e_prev)/Ts_sim;
        u(k)=max(0,min(2,u(k))); e_prev=e;
        u_buf=[u(k),u_buf(1:end-1)];
        if k<length(t_sim)
            y(k+1)=exp(-Ts_sim/T)*y(k)+K*(1-exp(-Ts_sim/T))*u_buf(round(tau/Ts_sim)+1);
        end
    end
    plot(t_sim, u, 'LineWidth', 1);
end
xlabel('时间 (s)'); ylabel('控制量');
title('控制器输出');
xlim([0, 1000]);

%% 输出对比表
fprintf('\n========== PID 整定方案对比表 ==========\n');
fprintf('%-12s %8s %10s %8s %10s %10s %8s %8s\n', ...
    '方案', 'Kp', 'Ki', 'Kd', 'Tr(s)', 'Ts(s)', 'σ(%)', '控制量');
fprintf('----------------------------------------------------------------------\n');
fprintf('%-12s %8.2f %10.4f %8.1f %10s %10s %8s %8.2f\n', ...
    '原始', Kp0, Ki0, Kd0, '-', '-', '-', max(u0));
for m = 1:size(results,1)
    fprintf('%-12s %8.2f %10.4f %8.1f %10.0f %10.1f %8.1f %8.2f\n', ...
        results{m,1}, results{m,2}, results{m,3}, results{m,4}, ...
        results{m,5}, results{m,6}, results{m,7}, results{m,8});
end
fprintf('----------------------------------------------------------------------\n');

%% 推荐方案
fprintf('\n=== 推荐方案 ===\n');
fprintf('推荐使用 自定义PI: Kp=%.2f, Ki=%.4f, D=0\n', Kp_manual, Ki_manual);
fprintf('  上升时间 Tr=%.0fs, 超调量 σ=%.1f%%\n', results{3,5}, results{3,7});
fprintf('\n修改 keshe1.slx 中的 PID Controller:\n');
fprintf('  Proportional: %.4f\n', Kp_manual);
fprintf('  Integral:     %.4f\n', Ki_manual);
fprintf('  Derivative:   0\n');
