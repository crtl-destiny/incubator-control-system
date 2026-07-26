"""
生化培养箱控制系统 — Python上位机监控程序
功能: 实时温度曲线、PID整定、传递函数辨识、串口通信
"""
import sys
import csv
import math
import time
import threading
import datetime
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("请先安装 pyserial: pip install pyserial")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("请先安装 numpy: pip install numpy")
    sys.exit(1)

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False


# ══════════════════════════════════════════════════════════════
#  串口数据解析
# ══════════════════════════════════════════════════════════════

class SensorData:
    def __init__(self):
        self.timestamp = time.time()
        self.current_temp = 0.0
        self.setpoint_temp = 0.0
        self.pid_output = 0.0
        self.heat_duty = 0.0
        self.cool_duty = 0.0
        self.run_time = 0.0

    @staticmethod
    def parse(line: str):
        d = SensorData()
        for part in line.split(","):
            part = part.strip()
            try:
                if part.startswith("T:"):
                    d.current_temp = float(part[2:])
                elif part.startswith("SET:"):
                    d.setpoint_temp = float(part[4:])
                elif part.startswith("PID:"):
                    d.pid_output = float(part[4:])
                elif part.startswith("HT:"):
                    d.heat_duty = float(part[3:].rstrip("%"))
                elif part.startswith("CL:"):
                    d.cool_duty = float(part[3:].rstrip("%"))
                elif part.startswith("RUN:"):
                    d.run_time = float(part[4:].rstrip("s"))
            except ValueError:
                continue
        return d


# ══════════════════════════════════════════════════════════════
#  传递函数辨识 (阶跃响应二点法)
# ══════════════════════════════════════════════════════════════

class TransferFunction:
    def __init__(self):
        self.K = 0.0
        self.T1 = 0.0
        self.tau = 0.0
        self.valid = False

    @staticmethod
    def identify(data_list):
        """从阶跃响应数据辨识一阶惯性+纯滞后模型 G(s)=K*e^(-tau*s)/(T1*s+1)"""
        if len(data_list) < 20:
            return None, "数据点不足(至少20个)"

        y0 = data_list[0].current_temp
        y_inf = data_list[-1].current_temp
        step_input = data_list[-1].setpoint_temp - data_list[0].setpoint_temp

        if abs(step_input) < 0.1:
            return None, "未检测到设定值阶跃变化(需 > 0.1℃)"

        rng = y_inf - y0
        y28 = y0 + 0.283 * rng
        y63 = y0 + 0.632 * rng

        t1 = t2 = 0.0
        found1 = found2 = False

        for i in range(1, len(data_list)):
            if not found1 and data_list[i].current_temp >= y28:
                frac = (y28 - data_list[i - 1].current_temp) / max(data_list[i].current_temp - data_list[i - 1].current_temp, 1e-9)
                t1 = data_list[i - 1].run_time + frac * (data_list[i].run_time - data_list[i - 1].run_time)
                found1 = True
            if not found2 and data_list[i].current_temp >= y63:
                frac = (y63 - data_list[i - 1].current_temp) / max(data_list[i].current_temp - data_list[i - 1].current_temp, 1e-9)
                t2 = data_list[i - 1].run_time + frac * (data_list[i].run_time - data_list[i - 1].run_time)
                found2 = True
                break

        if not found1 or not found2:
            return None, "未能找到完整的阶跃响应特征点"

        model = TransferFunction()
        model.T1 = 1.5 * (t2 - t1)
        model.tau = t2 - model.T1
        model.K = rng / step_input
        model.valid = True

        kp_zn = 0.6 / model.K * model.T1 / model.tau
        ti_zn = 0.5 * 2 * model.tau
        ki_zn = kp_zn / ti_zn if ti_zn > 0 else 0
        kd_zn = kp_zn * 0.125 * 2 * model.tau

        report = (
            f"辨识结果:\n"
            f"  G(s) = {model.K:.3f} * e^(-{model.tau:.1f}s) / ({model.T1:.1f}s + 1)\n\n"
            f"参数:\n"
            f"  增益 K          = {model.K:.3f}\n"
            f"  时间常数 T1     = {model.T1:.1f} s\n"
            f"  纯滞后时间 tau  = {model.tau:.1f} s\n\n"
            f"特征点:\n"
            f"  t1(28.3%) = {t1:.1f}s\n"
            f"  t2(63.2%) = {t2:.1f}s\n\n"
            f"Ziegler-Nichols 建议PID参数:\n"
            f"  Kp = {kp_zn:.3f}\n"
            f"  Ki = {ki_zn:.5f}\n"
            f"  Kd = {kd_zn:.3f}"
        )
        return model, report

    def simulate(self, t, step_amp):
        """仿真阶跃响应"""
        if not self.valid:
            return 0.0
        eff = t - self.tau
        if eff <= 0:
            return 0.0
        return step_amp * self.K * (1 - math.exp(-eff / self.T1))


# ══════════════════════════════════════════════════════════════
#  串口通信线程
# ══════════════════════════════════════════════════════════════

class SerialReader:
    def __init__(self):
        self.port = None
        self.connected = False
        self._thread = None
        self._running = False
        self.data_callback = None
        self.log_callback = None

    def connect(self, port_name, baud=115200):
        try:
            self.port = serial.Serial(port_name, baud, timeout=1)
            self.connected = True
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            return True
        except Exception as e:
            return str(e)

    def disconnect(self):
        self._running = False
        self.connected = False
        if self.port and self.port.is_open:
            self.port.close()
        self.port = None

    def send(self, cmd):
        if self.port and self.port.is_open:
            try:
                self.port.write(cmd.encode())
                if self.log_callback:
                    self.log_callback(f">>> {cmd.strip()}")
                return True
            except Exception as e:
                if self.log_callback:
                    self.log_callback(f"发送失败: {e}")
        return False

    def _read_loop(self):
        while self._running:
            try:
                if self.port and self.port.is_open and self.port.in_waiting:
                    line = self.port.readline().decode("utf-8", errors="ignore").strip()
                    if line and self.data_callback:
                        self.data_callback(line)
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)


# ══════════════════════════════════════════════════════════════
#  主界面
# ══════════════════════════════════════════════════════════════

class IncubatorApp:
    MAX_POINTS = 3000

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("生化培养箱控制系统 — 上位机监控")
        self.root.geometry("1300x850")
        self.root.minsize(1000, 600)

        self.reader = SerialReader()
        self.reader.data_callback = self._on_data
        self.reader.log_callback = self._log

        self.data_log = []
        self._collecting = False
        self._sim_series_added = False

        self._build_ui()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ─── UI 构建 ──────────────────────────────────────────

    def _build_ui(self):
        style = ttk.Style()
        style.configure("Status.TLabel", foreground="gray", font=("Microsoft YaHei", 10))

        # ── 顶部工具栏 ──
        toolbar = ttk.Frame(self.root, padding=6)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(toolbar, text="串口:").pack(side=tk.LEFT)
        self.cmb_port = ttk.Combobox(toolbar, width=12, state="readonly")
        self.cmb_port.pack(side=tk.LEFT, padx=(4, 8))

        ttk.Button(toolbar, text="刷新", width=5, command=self._refresh_ports).pack(side=tk.LEFT, padx=2)

        self.btn_connect = ttk.Button(toolbar, text="连接", width=6, command=self._toggle_connect)
        self.btn_connect.pack(side=tk.LEFT, padx=4)

        self.lbl_status = ttk.Label(toolbar, text="● 未连接", style="Status.TLabel")
        self.lbl_status.pack(side=tk.LEFT, padx=8)

        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Button(toolbar, text="开始辨识", width=8, command=self._do_identify).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="清空数据", width=8, command=self._clear_data).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="导出CSV", width=8, command=self._export_csv).pack(side=tk.LEFT, padx=2)

        # ── 主区域 (左图 + 右控件) ──
        main_frame = ttk.Frame(self.root)
        main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=4)

        # 左侧: 图表
        chart_frame = ttk.LabelFrame(main_frame, text="实时曲线", padding=4)
        chart_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(8, 5), dpi=100, facecolor="white")
        self.ax_temp = self.fig.add_subplot(111)
        self.ax_pid = self.ax_temp.twinx()

        self.ax_temp.set_xlabel("时间 (s)")
        self.ax_temp.set_ylabel("温度 (℃)", color="blue")
        self.ax_pid.set_ylabel("PID输出 (%)", color="green")
        self.ax_temp.grid(True, alpha=0.3)
        self.ax_temp.set_ylim(0, 60)
        self.ax_pid.set_ylim(-110, 110)

        self.canvas = FigureCanvasTkAgg(self.fig, master=chart_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.line_temp, = self.ax_temp.plot([], [], "b-", linewidth=1.5, label="当前温度")
        self.line_set, = self.ax_temp.plot([], [], "r--", linewidth=1, label="设定温度")
        self.line_pid, = self.ax_pid.plot([], [], "g-", linewidth=1, alpha=0.7, label="PID输出")
        self.line_sim = None

        # 右侧: 控件面板
        right_frame = ttk.Frame(main_frame, width=340)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(6, 0))
        right_frame.pack_propagate(False)

        # ── 参数显示 ──
        info_frame = ttk.LabelFrame(right_frame, text="实时数据", padding=8)
        info_frame.pack(fill=tk.X, pady=(0, 6))

        self.var_temp = tk.StringVar(value="--.- ℃")
        self.var_set = tk.StringVar(value="--.- ℃")
        self.var_pid = tk.StringVar(value="--.- %")
        self.var_heat = tk.StringVar(value="-- %")
        self.var_cool = tk.StringVar(value="-- %")
        self.var_run = tk.StringVar(value="-- s")

        for i, (label, var) in enumerate([
            ("当前温度", self.var_temp), ("设定温度", self.var_set),
            ("PID输出", self.var_pid), ("加热占空比", self.var_heat),
            ("制冷占空比", self.var_cool), ("运行时间", self.var_run)
        ]):
            row, col = divmod(i, 2)
            ttk.Label(info_frame, text=label, font=("Microsoft YaHei", 9)).grid(row=row * 2, column=col, sticky=tk.W)
            ttk.Label(info_frame, textvariable=var, font=("Consolas", 11, "bold")).grid(row=row * 2 + 1, column=col, sticky=tk.W)

        # ── 参数设定 ──
        set_frame = ttk.LabelFrame(right_frame, text="参数设定", padding=8)
        set_frame.pack(fill=tk.X, pady=(0, 6))

        ttk.Label(set_frame, text="设定温度 (5~50℃):").grid(row=0, column=0, sticky=tk.W)
        self.spn_setpoint = ttk.Spinbox(set_frame, from_=5.0, to=50.0, increment=0.5, width=8, format="%.1f")
        self.spn_setpoint.delete(0, tk.END)
        self.spn_setpoint.insert(0, "37.0")
        self.spn_setpoint.grid(row=0, column=1, padx=4)

        ttk.Button(set_frame, text="发送设定值", command=self._send_setpoint).grid(row=1, column=0, columnspan=2, sticky=tk.EW, pady=4)

        ttk.Label(set_frame, text="手动命令:").grid(row=2, column=0, sticky=tk.W)
        self.ent_cmd = ttk.Entry(set_frame, width=20)
        self.ent_cmd.grid(row=2, column=1, padx=4, sticky=tk.EW)
        self.ent_cmd.bind("<Return>", lambda e: self._send_manual())

        ttk.Button(set_frame, text="发送", width=6, command=self._send_manual).grid(row=3, column=1, sticky=tk.E, pady=2)

        # ── 辨识结果 ──
        id_frame = ttk.LabelFrame(right_frame, text="模型辨识结果", padding=8)
        id_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 0))

        self.txt_result = tk.Text(id_frame, height=10, font=("Consolas", 9), wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(id_frame, command=self.txt_result.yview)
        self.txt_result.configure(yscrollcommand=scrollbar.set)
        self.txt_result.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # ── 底部日志 ──
        log_frame = ttk.LabelFrame(self.root, text="通信日志", padding=4)
        log_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=4)

        self.log_list = tk.Listbox(log_frame, height=4, font=("Consolas", 8))
        self.log_list.pack(fill=tk.X)

    # ─── 串口操作 ──────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb_port["values"] = ports
        if ports:
            self.cmb_port.set(ports[0])
        else:
            self.cmb_port.set("COM3")

    def _toggle_connect(self):
        if self.reader.connected:
            self.reader.disconnect()
            self.btn_connect.config(text="连接")
            self.lbl_status.config(text="● 未连接", foreground="gray")
            self._log("串口已断开")
        else:
            port = self.cmb_port.get()
            if not port:
                messagebox.showwarning("提示", "请先选择串口")
                return
            err = self.reader.connect(port)
            if err is True:
                self.btn_connect.config(text="断开")
                self.lbl_status.config(text=f"● 已连接 ({port})", foreground="green")
                self._log(f"串口已打开: {port}")
            else:
                messagebox.showerror("错误", f"无法打开串口:\n{err}")

    # ─── 数据处理 ──────────────────────────────────────────

    def _on_data(self, line):
        d = SensorData.parse(line)
        if not d:
            return
        self.data_log.append(d)
        if len(self.data_log) > self.MAX_POINTS:
            self.data_log = self.data_log[-self.MAX_POINTS:]

        # 更新标签
        self.var_temp.set(f"{d.current_temp:.1f} ℃")
        self.var_set.set(f"{d.setpoint_temp:.1f} ℃")
        self.var_pid.set(f"{d.pid_output:.1f} %")
        self.var_heat.set(f"{d.heat_duty:.0f} %")
        self.var_cool.set(f"{d.cool_duty:.0f} %")
        self.var_run.set(f"{d.run_time:.0f} s")

        # 更新图表
        self._update_chart(d)
        self._log(line)

    def _update_chart(self, d):
        self.line_temp.set_xdata([p.run_time for p in self.data_log])
        self.line_temp.set_ydata([p.current_temp for p in self.data_log])

        self.line_set.set_xdata([p.run_time for p in self.data_log])
        self.line_set.set_ydata([p.setpoint_temp for p in self.data_log])

        self.line_pid.set_xdata([p.run_time for p in self.data_log])
        self.line_pid.set_ydata([p.pid_output for p in self.data_log])

        if self.data_log:
            t_max = self.data_log[-1].run_time
            self.ax_temp.set_xlim(max(0, t_max - 120), max(120, t_max + 10))

        self.canvas.draw_idle()

    # ─── 命令发送 ──────────────────────────────────────────

    def _send_setpoint(self):
        val = self.spn_setpoint.get()
        try:
            temp = float(val)
            if temp < 5.0 or temp > 50.0:
                messagebox.showwarning("提示", "温度范围: 5.0 ~ 50.0 ℃")
                return
            self.reader.send(f"SET:{temp:.1f}\r\n")
        except ValueError:
            messagebox.showwarning("提示", "请输入有效的温度值")

    def _send_manual(self):
        cmd = self.ent_cmd.get().strip()
        if cmd:
            self.reader.send(cmd + "\r\n")
            self.ent_cmd.delete(0, tk.END)

    # ─── 模型辨识 ──────────────────────────────────────────

    def _do_identify(self):
        if len(self.data_log) < 20:
            messagebox.showwarning("提示", "数据点不足(至少20个)\n请连接后等待数据采集")
            return

        step_data = self._find_step_region()
        if not step_data:
            step_data = self.data_log

        model, report = TransferFunctionIdentification.identify(step_data)
        self.txt_result.delete("1.0", tk.END)
        self.txt_result.insert("1.0", report)

        if model and model.valid:
            self._overlay_simulation(model)

    def _find_step_region(self):
        if len(self.data_log) < 20:
            return None
        max_delta = 0
        step_idx = 0
        for i in range(10, len(self.data_log) - 10):
            delta = abs(self.data_log[i].setpoint_temp - self.data_log[i - 1].setpoint_temp)
            if delta > max_delta:
                max_delta = delta
                step_idx = i
        if max_delta < 0.5:
            return None
        start = max(0, step_idx - 10)
        return self.data_log[start:]

    def _overlay_simulation(self, model):
        if self._sim_series_added and self.line_sim:
            self.line_sim.remove()
            self.line_sim = None
            self._sim_series_added = False

        if not self.data_log:
            return

        step_amp = self.data_log[-1].setpoint_temp - self.data_log[0].setpoint_temp
        if abs(step_amp) < 0.1:
            step_amp = 10.0

        t0 = self.data_log[0].run_time
        t_end = self.data_log[-1].run_time + 200
        if t_end > t0 + 1200:
            t_end = t0 + 1200

        ts = np.linspace(t0, t_end, 500)
        vals = [model.simulate(t - t0, step_amp) + self.data_log[0].current_temp for t in ts]

        self.line_sim, = self.ax_temp.plot(ts, vals, "orange", linewidth=2, linestyle="--", label="辨识模型")
        self._sim_series_added = True
        self.canvas.draw_idle()

    # ─── 数据导出 ──────────────────────────────────────────

    def _export_csv(self):
        if not self.data_log:
            messagebox.showinfo("提示", "没有数据可导出")
            return
        fn = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            initialfile=f"incubator_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
        )
        if not fn:
            return
        with open(fn, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["Time_s", "CurrentTemp_C", "SetpointTemp_C", "PIDOutput", "HeatDuty_Pct", "CoolDuty_Pct"])
            for d in self.data_log:
                w.writerow([f"{d.run_time:.1f}", f"{d.current_temp:.2f}", f"{d.setpoint_temp:.2f}",
                            f"{d.pid_output:.2f}", f"{d.heat_duty:.0f}", f"{d.cool_duty:.0f}"])
        messagebox.showinfo("导出成功", f"已导出 {len(self.data_log)} 行到:\n{fn}")

    def _clear_data(self):
        self.data_log.clear()
        self.line_temp.set_data([], [])
        self.line_set.set_data([], [])
        self.line_pid.set_data([], [])
        if self.line_sim:
            self.line_sim.remove()
            self.line_sim = None
            self._sim_series_added = False
        self.ax_temp.set_xlim(0, 120)
        self.canvas.draw_idle()
        self.txt_result.delete("1.0", tk.END)
        self.log_list.delete(0, tk.END)

    # ─── 工具方法 ──────────────────────────────────────────

    def _log(self, msg):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_list.insert(tk.END, f"[{ts}] {msg}")
        if self.log_list.size() > 500:
            self.log_list.delete(0, 100)
        self.log_list.see(tk.END)

    def _on_close(self):
        self.reader.disconnect()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# 将辨识类重命名为与C#版本一致
TransferFunctionIdentification = TransferFunction


# ══════════════════════════════════════════════════════════════
#  入口
# ══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = IncubatorApp()
    app.run()
