# 生化培养箱控制系统 — 软件流程图

> 规范：单主线串行架构，Y左分支/N直下，循环回流到信息上传。

---

## 图1.1 主程序流程

```mermaid
%%{init: {
  'theme': 'neutral',
  'themeVariables': {
    'primaryColor': '#FFFFFF',
    'primaryTextColor': '#000000',
    'primaryBorderColor': '#000000',
    'lineColor': '#333333',
    'secondaryColor': '#F5F5F5',
    'tertiaryColor': '#FAFAFA',
    'fontFamily': 'Times New Roman, SimSun, serif',
    'fontSize': '12px'
  },
  'flowchart': {
    'curve': 'basis',
    'padding': 12,
    'nodeSpacing': 25,
    'rankSpacing': 35,
    'htmlLabels': true
  }
}}%%
graph LR
  Start(["开始"]) --> Init["模块初始化<div style='font-size:10px;color:#555'>• HAL/时钟/外设初始化<br/>• OLED/DS18B20/键盘初始化<br/>• PID/控制模块/串口初始化</div>"]
  Init --> Upload["信息上传"]
  Upload --> KeyCheck{"按键检测"}
  KeyCheck -->|Y| KeyProc["按键处理"]
  KeyProc --> OLEDRef["OLED刷新"]
  OLEDRef --> CycleJug
  KeyCheck -->|N| CycleJug{"控制周期到？<div style='font-size:10px;color:#555'>(200ms)</div>"}

  CycleJug -->|Y| CycleTask["控制周期任务<div style='font-size:10px;color:#555'>• DS18B20温度采集<br/>• 蜂鸣器安全守卫<br/>• 三段式报警检测<br/>• PID控制计算与输出<br/>• OLED刷新显示</div>"]
  CycleJug -->|N| UartJug
  CycleTask --> UartJug{"串口上报周期到？<div style='font-size:10px;color:#555'>(1s)</div>"}

  UartJug -->|Y| UartReport["串口数据上报"]
  UartJug -->|N| UartRx["串口接收解析"]
  UartReport --> UartRx
  UartRx --> Upload

  classDef start fill:#000,color:#FFF,stroke:#000,stroke-width:2px
  classDef process fill:#FFF,stroke:#000,stroke-width:1.5px
  classDef judge fill:#FFF,stroke:#000,stroke-width:1.5px

  class Start start
  class Init,Upload,KeyProc,OLEDRef,CycleTask,UartReport,UartRx process
  class KeyCheck,CycleJug,UartJug judge
```

> **图1.1 主程序流程图（水平布局）**。系统上电初始化后进入无限循环，依次执行按键检测、控制周期任务（200ms）、串口上报（1s），末尾回流至信息上传。

---

## 图1.2 系统状态机

```mermaid
%%{init: {
  'theme': 'neutral',
  'themeVariables': {
    'primaryColor': '#FFFFFF',
    'primaryTextColor': '#000000',
    'primaryBorderColor': '#000000',
    'lineColor': '#333333',
    'secondaryColor': '#F5F5F5',
    'tertiaryColor': '#FAFAFA',
    'fontFamily': 'Times New Roman, SimSun, serif',
    'fontSize': '12px'
  },
  'flowchart': {
    'curve': 'basis',
    'padding': 14,
    'nodeSpacing': 30,
    'rankSpacing': 40,
    'htmlLabels': true
  }
}}%%
graph LR
  Start(["[*]"]) -->|上电| Idle["空闲态 IDLE<div style='font-size:10px;color:#555'>不控温</div>"]
  Idle -->|按 *| Setting["设定态 SETTING<div style='font-size:10px;color:#555'>输入温度</div>"]
  Idle -->|按 # 直启| Running["运行态 RUNNING<div style='font-size:10px;color:#555'>PID控温</div>"]
  Setting -->|数字/退格| Setting
  Setting -->|按 # 确认| Running
  Running -->|按 # 停止| Idle
  Running -->|偏差>2℃ 卡住60s| Alarm["报警态 ALARM<div style='font-size:10px;color:#555'>蜂鸣器响</div>"]
  Alarm -->|偏差≤2℃ 恢复| Running

  classDef start fill:#000,color:#FFF,stroke:#000,stroke-width:2px
  classDef state fill:#FFF,stroke:#000,stroke-width:1.5px
  classDef alarm fill:#FFF,stroke:#000,stroke-width:2px

  class Start start
  class Idle,Setting,Running state
  class Alarm alarm
```

> **图1.2 系统状态机**。包含空闲、设定、运行、报警四种状态，通过按键和条件事件驱动状态转移。

---

## 图1.3 PID控制算法

```mermaid
%%{init: {
  'theme': 'neutral',
  'themeVariables': {
    'primaryColor': '#FFFFFF',
    'primaryTextColor': '#000000',
    'primaryBorderColor': '#000000',
    'lineColor': '#333333',
    'secondaryColor': '#F5F5F5',
    'tertiaryColor': '#FAFAFA',
    'fontFamily': 'Times New Roman, SimSun, serif',
    'fontSize': '12px'
  },
  'flowchart': {
    'curve': 'basis',
    'padding': 12,
    'nodeSpacing': 25,
    'rankSpacing': 35,
    'htmlLabels': true
  }
}}%%
graph TD
  Start(["PID计算"]) --> CalcE["计算偏差 e = SP - PV"]
  CalcE --> Sep{"|e| &gt; 3℃ ？"}
  Sep -->|Y| ClearI["积分项清零 i_term = 0"]
  Sep -->|N| AccI["积分累加 integral += e × Ts<div style='font-size:10px;color:#555'>限幅保护</div>"]
  AccI --> CalcI["计算 i_term = Ki × integral"]

  ClearI --> CalcP["计算 p_term = Kp × e"]
  CalcI --> CalcP

  CalcP --> CalcD["计算 d_term<div style='font-size:10px;color:#555'>-Kd × (y(k)-y(k-1))/Ts<br/>微分对测量值</div>"]
  CalcD --> Sum["求和 u = p + i + d"]

  Sum --> Sat{"输出超限幅？"}
  Sat -->|Y| Clip["限幅到 ±100<div style='font-size:10px;color:#555'>反方向回退积分</div>"]
  Sat -->|N| Return["返回 u (-100 ~ +100)"]
  Clip --> Return

  classDef start fill:#000,color:#FFF,stroke:#000,stroke-width:2px
  classDef process fill:#FFF,stroke:#000,stroke-width:1.5px
  classDef judge fill:#FFF,stroke:#000,stroke-width:1.5px

  class Start start
  class CalcE,ClearI,AccI,CalcI,CalcP,CalcD,Sum,Clip process
  class Sep,Sat judge
  class Return process
```

> **图1.3 PID控制算法**。位置式PID，偏差>3℃时积分清零防饱和，微分作用于测量值避冲击，输出限幅±100并抗积分回退。

---

## 图1.4 DS18B20温度采集（非阻塞策略）

```mermaid
%%{init: {
  'theme': 'neutral',
  'themeVariables': {
    'primaryColor': '#FFFFFF',
    'primaryTextColor': '#000000',
    'primaryBorderColor': '#000000',
    'lineColor': '#333333',
    'secondaryColor': '#F5F5F5',
    'tertiaryColor': '#FAFAFA',
    'fontFamily': 'Times New Roman, SimSun, serif',
    'fontSize': '12px'
  },
  'flowchart': {
    'curve': 'basis',
    'padding': 12,
    'nodeSpacing': 25,
    'rankSpacing': 30,
    'htmlLabels': true
  }
}}%%
graph TD
  Start(["温度采集"]) --> CntCheck{"采集周期到？<div style='font-size:10px;color:#555'>(800ms)</div>"}
  CntCheck -->|N| Skip["跳过本次采集"]
  CntCheck -->|Y| ReadCmd["发读取命令<div style='font-size:10px;color:#555'>复位→0xCC→0xBE</div>"]
  ReadCmd --> Recv["接收温度数据<div style='font-size:10px;color:#555'>2字节 raw</div>"]
  Recv --> Calc["计算温度值<div style='font-size:10px;color:#555'>temp = raw × 0.0625</div>"]
  Calc --> StartCmd["启动新转换<div style='font-size:10px;color:#555'>复位→0xCC→0x44</div>"]
  StartCmd --> Return

  Skip --> Return(["返回"])

  classDef start fill:#000,color:#FFF,stroke:#000,stroke-width:2px
  classDef process fill:#FFF,stroke:#000,stroke-width:1.5px
  classDef judge fill:#FFF,stroke:#000,stroke-width:1.5px

  class Start start
  class ReadCmd,Recv,Calc,StartCmd,Skip process
  class CntCheck judge
  class Return process
```

> **图1.4 DS18B20温度采集（非阻塞策略）**。每800ms启动一次转换，非采集周期跳过，避免阻塞主循环。

---

## 图1.5 矩阵键盘扫描

```mermaid
%%{init: {
  'theme': 'neutral',
  'themeVariables': {
    'primaryColor': '#FFFFFF',
    'primaryTextColor': '#000000',
    'primaryBorderColor': '#000000',
    'lineColor': '#333333',
    'secondaryColor': '#F5F5F5',
    'tertiaryColor': '#FAFAFA',
    'fontFamily': 'Times New Roman, SimSun, serif',
    'fontSize': '12px'
  },
  'flowchart': {
    'curve': 'basis',
    'padding': 12,
    'nodeSpacing': 25,
    'rankSpacing': 30,
    'htmlLabels': true
  }
}}%%
graph TD
  Start(["按键扫描"]) --> DebCheck{"去抖50ms中？"}
  DebCheck -->|Y| None1["返回无按键"]
  DebCheck -->|N| LineScan["逐行扫描<div style='font-size:10px;color:#555'>行置低 → 读列</div>"]
  LineScan --> PressCheck{"有键按下？"}
  PressCheck -->|N| None1
  PressCheck -->|Y| DupCheck{"与上次键值相同？"}
  DupCheck -->|Y| None1
  DupCheck -->|N| Save["记录键值和时间戳"]
  Save --> RetKey["返回键值"]

  classDef start fill:#000,color:#FFF,stroke:#000,stroke-width:2px
  classDef process fill:#FFF,stroke:#000,stroke-width:1.5px
  classDef judge fill:#FFF,stroke:#000,stroke-width:1.5px

  class Start start
  class LineScan,Save,None1 process
  class DebCheck,PressCheck,DupCheck judge
  class RetKey process
```

> **图1.5 矩阵键盘扫描**。50ms去抖 + 连续重复键过滤，确保按键输入稳定。

---

## 图1.6 三段式报警检测

```mermaid
%%{init: {
  'theme': 'neutral',
  'themeVariables': {
    'primaryColor': '#FFFFFF',
    'primaryTextColor': '#000000',
    'primaryBorderColor': '#000000',
    'lineColor': '#333333',
    'secondaryColor': '#F5F5F5',
    'tertiaryColor': '#FAFAFA',
    'fontFamily': 'Times New Roman, SimSun, serif',
    'fontSize': '12px'
  },
  'flowchart': {
    'curve': 'basis',
    'padding': 12,
    'nodeSpacing': 20,
    'rankSpacing': 30,
    'htmlLabels': true
  }
}}%%
graph TD
  Start(["报警检测"]) --> RunCheck{"运行/报警态？"}
  RunCheck -->|N| BuzzerOff["关蜂鸣器"]
  BuzzerOff --> Ret

  RunCheck -->|Y| Phase1Check{"启动 &lt; 10min ？"}
  Phase1Check -->|Y| BuzzerOff

  Phase1Check -->|N| DevCheck{"|偏差| ≤ 2℃ ？"}
  DevCheck -->|Y| ClearAll["清计数器、关蜂鸣器<div style='font-size:10px;color:#555'>恢复 RUNNING</div>"]
  ClearAll --> Ret

  DevCheck -->|N| ImproveCheck{"偏差在改善？"}
  ImproveCheck -->|Y| ResetTimer["清卡住计时器"]
  ResetTimer --> Ret

  ImproveCheck -->|N| IncTimer["卡住计时器++"]
  IncTimer --> TimeCheck{"卡住 ≥ 60s ？"}
  TimeCheck -->|N| Ret
  TimeCheck -->|Y| SetAlarm["置 ALARM 状态<div style='font-size:10px;color:#555'>蜂鸣器响</div>"]
  SetAlarm --> Ret(["返回"])

  classDef start fill:#000,color:#FFF,stroke:#000,stroke-width:2px
  classDef process fill:#FFF,stroke:#000,stroke-width:1.5px
  classDef judge fill:#FFF,stroke:#000,stroke-width:1.5px

  class Start start
  class BuzzerOff,ClearAll,ResetTimer,IncTimer,SetAlarm process
  class RunCheck,Phase1Check,DevCheck,ImproveCheck,TimeCheck judge
  class Ret process
```

> **图1.6 三段式报警检测**。阶段一（启动10min内）不报警；阶段二偏差≤2℃清除报警；阶段三偏差持续≥2℃且60s无改善触发报警。

---

## 图1.7 BTS7960控制区段映射

```mermaid
%%{init: {
  'theme': 'neutral',
  'themeVariables': {
    'primaryColor': '#FFFFFF',
    'primaryTextColor': '#000000',
    'primaryBorderColor': '#000000',
    'lineColor': '#333333',
    'secondaryColor': '#F5F5F5',
    'tertiaryColor': '#FAFAFA',
    'fontFamily': 'Times New Roman, SimSun, serif',
    'fontSize': '12px'
  },
  'flowchart': {
    'curve': 'basis',
    'padding': 12,
    'nodeSpacing': 25,
    'rankSpacing': 30,
    'htmlLabels': true
  }
}}%%
graph TD
  Start(["控制输出"]) --> Limit["限幅 ±100"]
  Limit --> HeatCheck{"pid_out &gt; +5% ？"}
  HeatCheck -->|Y| HeatMode["加热模式<div style='font-size:10px;color:#555'>IN1=PWM, IN2=LOW</div>"]
  HeatCheck -->|N| CoolCheck{"pid_out &lt; -5% ？"}
  CoolCheck -->|Y| CoolMode["制冷模式<div style='font-size:10px;color:#555'>IN1=LOW, IN2=PWM</div>"]
  CoolCheck -->|N| IdleMode["空闲模式<div style='font-size:10px;color:#555'>IN1=LOW, IN2=LOW</div>"]
  HeatMode --> WritePWM["写 TIM3 CCR"]
  CoolMode --> WritePWM
  IdleMode --> WritePWM
  WritePWM --> Ret(["返回"])

  classDef start fill:#000,color:#FFF,stroke:#000,stroke-width:2px
  classDef process fill:#FFF,stroke:#000,stroke-width:1.5px
  classDef judge fill:#FFF,stroke:#000,stroke-width:1.5px

  class Start start
  class Limit,HeatMode,CoolMode,IdleMode,WritePWM process
  class HeatCheck,CoolCheck judge
  class Ret process
```

> **图1.7 BTS7960 H桥控制区段映射**。PID输出经±100限幅后，按死区±5%划分为加热/空闲/制冷三区段，通过TIM3输出PWM。

---

## 图1.8 Python上位机软件架构

```mermaid
%%{init: {
  'theme': 'neutral',
  'themeVariables': {
    'primaryColor': '#FFFFFF',
    'primaryTextColor': '#000000',
    'primaryBorderColor': '#000000',
    'lineColor': '#333333',
    'secondaryColor': '#F5F5F5',
    'tertiaryColor': '#FAFAFA',
    'fontFamily': 'Times New Roman, SimSun, serif',
    'fontSize': '12px'
  },
  'flowchart': {
    'curve': 'basis',
    'padding': 14,
    'nodeSpacing': 30,
    'rankSpacing': 40,
    'htmlLabels': true
  }
}}%%
graph TD
  subgraph 下位机
    STM32["STM32F103"]
    UART["USART1<div style='font-size:10px;color:#555'>115200 bps</div>"]
  end

  subgraph 通信层
    Serial["串口读取线程"]
    Parser["数据帧解析"]
  end

  subgraph 上位机
    GUI["GUI主界面"]
    Chart["实时曲线"]
    Panel["数据显示"]
    CmdPanel["参数设定"]
    Identify["传递函数辨识"]
    LogPanel["通信日志"]
  end

  STM32 --> UART --> Serial --> Parser --> GUI
  GUI --> Chart & Panel & LogPanel
  Parser --> Identify --> Chart
  CmdPanel --> Serial --> UART --> STM32

  classDef box fill:#F5F5F5,stroke:#000,stroke-width:1.5px
  classDef node fill:#FFF,stroke:#000,stroke-width:1.5px

  class STM32,UART,Serial,Parser box
  class GUI,Chart,Panel,CmdPanel,Identify,LogPanel node
```

> **图1.8 Python上位机软件架构**。上位机通过串口与下位机通信，实现实时数据显示、参数设定、传递函数辨识及数据记录功能。
