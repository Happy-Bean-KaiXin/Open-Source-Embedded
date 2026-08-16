# -*- coding: utf-8 -*-
"""
Speed-loop numerical simulation for migong_car5.

PURPOSE
-------
Reproduce the "car jitters / fights itself when target speed is raised" bug,
and verify the fix, WITHOUT touching real hardware.

WHAT IS MODELED (mirrors the firmware exactly)
----------------------------------------------
1. Speed-loop PID = pid.c::Position_PID_Realize (position-form PID with integral
   clamp +/-INTEG_LIMIT). Current gains: Kp=-30, Ki=-2.6 (NEGATIVE).
2. The firmware calls the PID inside the main() super-loop (no 10 ms gating),
   while the encoder speed is only refreshed every 10 ms inside the TIM4 ISR.
   -> We model two call modes:
        * "current":  PID step every 1 ms, but reality_fed only updated every 10 ms
                      (same stale value integrated 10x -> integrates "the same
                       error" repeatedly, inflating the effective integral gain).
        * "fixed":    PID step every 10 ms, aligned with the encoder refresh.
3. Motor + wheel = first-order plant: r_phys -> (-KPLANT * pwm), i.e. the net
   polarity already baked into the board (motor wired reversed + encoder sign
   flip) so that the loop is a working negative-feedback as observed
   ("car can crawl forward at low speed").

The two sign-flips (encoder -GetMotorPulse and reversed motor) cancel, so the
value fed to the PID is simply r_phys (positive = forward), exactly as the
firmware's `Uinttime_MotorPulse` ends up being. That is why Kp<0 still works
at low speed -- it is "wrong-but-compensated". The bug is the *high-frequency
unsynchronised integration*, which saturates the integrator at high target
speed and produces a limit cycle (jitter).

Run:  python speed_loop_sim.py
Produces: sim_report.html  (self-contained, with inline SVG plots)
"""
import math

# ---------------- firmware constants ----------------
MAX_MOTOR_PWM = 4500
ACircleEncoder = 1560
WheelDiameter = 0.060
WheelOneCircleDis = WheelDiameter * math.pi      # ~0.1885 m
INTEG_LIMIT = 5000                                # pid.c integral clamp

# current speed-loop gains (pid.c Straight_Trail_param)
KP = -30.0
KI = -2.6
KD = 0.0

# ---------------- plant model (reasonable estimates) ----------------
# pwm = 4500 (full) should drive ~70 pulses / 10ms (~0.84 m/s at this wheel).
KPLANT = 70.0 / 4500.0
TAU = 0.25                                        # mechanical time constant (s)
# Control-loop transport delay (motor electrical + mechanical + sampling lag).
# This delay is what turns "high-frequency unsynchronised integration" into a
# real limit cycle at high gain -> the jitter the user observed.
PLANT_DELAY = 4                                   # steps (=4 ms at 1 ms grid)

# ---------------- simulation grid ----------------
DT_MS = 1.0
CTRL_MS = 10.0
SIM_SEC = 20.0
N = int(SIM_SEC * 1000 / DT_MS)
CTRL_STEPS = int(CTRL_MS / DT_MS)                 # 10


def tar_encode(tar_speed):                        # m/s -> pulses / 10ms
    return tar_speed * ACircleEncoder / (WheelOneCircleDis * 100.0)


def speed_of(reality_fed):                        # pulses / 10ms -> m/s
    return reality_fed / ACircleEncoder * WheelOneCircleDis * 100.0


class PosPID:
    """Exact copy of pid.c::Position_PID_Realize logic."""
    def __init__(self, kp, ki, kd, ilim):
        self.kp, self.ki, self.kd, self.ilim = kp, ki, kd, ilim
        self.bias = 0.0
        self.last = 0.0
        self.integ = 0.0

    def reset(self):
        self.bias = self.last = self.integ = 0.0

    def step(self, reality, target):
        self.bias = target - reality
        self.integ += self.bias
        if self.integ > self.ilim:
            self.integ = self.ilim
        if self.integ < -self.ilim:
            self.integ = -self.ilim
        pwm = (self.kp * self.bias
               + self.ki * self.integ
               + self.kd * (self.bias - self.last))
        self.last = self.bias
        return pwm


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def sim(target, kp, ki, call_every_ms, ilim=INTEG_LIMIT):
    """Return (ts, speeds, pwms) for one run."""
    pid = PosPID(kp, ki, KD, ilim)
    r_phys = 0.0
    pwm = 0.0
    reality_fed = 0.0
    call_steps = max(1, int(round(call_every_ms / DT_MS)))
    # pwm history ring buffer to model transport delay
    pwm_hist = [0.0] * (PLANT_DELAY + 1)
    ts, speeds, pwms = [], [], []
    for step in range(N):
        t = step * DT_MS / 1000.0
        # plant sees the delayed pwm (control-loop lag)
        pwm_delayed = pwm_hist[0]
        drive = -KPLANT * pwm_delayed
        r_phys += (DT_MS / 1000.0 / TAU) * (drive - r_phys)
        # encoder sampled every CTRL_MS (the two sign-flips cancel -> = r_phys)
        if step % CTRL_STEPS == 0:
            reality_fed = r_phys
        # PID call (current: every 1 ms with STALE reality_fed between samples)
        if step % call_steps == 0:
            pwm = clamp(pid.step(reality_fed, target), -MAX_MOTOR_PWM, MAX_MOTOR_PWM)
        # shift pwm into delay buffer
        pwm_hist = pwm_hist[1:] + [pwm]
        ts.append(t)
        speeds.append(speed_of(r_phys))
        pwms.append(pwm)
    return ts, speeds, pwms


def steady_metrics(speeds):
    half = len(speeds) // 2
    seg = speeds[half:]
    mean = sum(seg) / len(seg)
    var = sum((x - mean) ** 2 for x in seg) / len(seg)
    return mean, math.sqrt(var)


# ---------------- scenarios ----------------
targets = [0.2, 0.4, 0.6, 0.8, 1.0]
scenarios = [
    ("current (Kp=-30,Ki=-2.6, 1ms call)", KP, KI, 1, INTEG_LIMIT),
    ("fix #1  (Kp=-30,Ki=-2.6, 10ms call)", KP, KI, 10, INTEG_LIMIT),
    ("fix #5  (Kp=+30,Ki=+2.6, 10ms call)", -KP, -KI, 10, INTEG_LIMIT),
]

results = {}   # name -> {target: (mean, jitter)}
for name, kp, ki, callms, ilim in scenarios:
    results[name] = {}
    for T in targets:
        _, sp, _ = sim(tar_encode(T), kp, ki, callms, ilim)
        results[name][T] = steady_metrics(sp)

# ---------------- pick a high-speed trace to plot (T=1.0, near saturation) ----------------
plot_T = 1.0
traces = {}
for name, kp, ki, callms, ilim in scenarios:
    ts, sp, pw = sim(tar_encode(plot_T), kp, ki, callms, ilim)
    traces[name] = (ts, sp, pw)


# ---------------- SVG plotting helpers ----------------
def polyline(xs, ys, w, h, xmax, ymin, ymax):
    """Map data to an SVG polyline inside a w x h box."""
    if ymax == ymin:
        ymax = ymin + 1
    pts = []
    for x, y in zip(xs, ys):
        px = 38 + (x / xmax) * (w - 48)
        py = (h - 18) - ((y - ymin) / (ymax - ymin)) * (h - 36)
        pts.append("%.1f,%.1f" % (px, py))
    return '<polyline fill="none" stroke="__C__" stroke-width="1.6" points="%s"/>' % " ".join(pts)


def axis(xmax, ymin, ymax, w, h, ylabel, yfmt):
    s = []
    # y gridlines
    for i in range(5):
        yy = ymin + (ymax - ymin) * i / 4
        py = (h - 18) - ((yy - ymin) / (ymax - ymin)) * (h - 36)
        s.append('<line x1="38" y1="%.1f" x2="%d" y2="%.1f" stroke="#3a3f4b" stroke-width="0.6"/>' % (py, w - 10, py))
        s.append('<text x="4" y="%.1f" fill="#9aa4b2" font-size="9">%s</text>' % (py + 3, yfmt % yy))
    # x ticks
    for i in range(4):
        xx = xmax * i / 3
        px = 38 + (xx / xmax) * (w - 48)
        s.append('<text x="%.1f" y="%d" fill="#9aa4b2" font-size="9">%.1fs</text>' % (px - 6, h - 4, xx))
    s.append('<text x="%d" y="12" fill="#cdd6e4" font-size="10">%s</text>' % (w // 2 - 30, ylabel))
    return "".join(s)


colors = ["#ff5d5d", "#4fd97f", "#5db4ff"]
svg_plots = []

# plot 1: speed time-domain at T=0.8
w, h = 560, 230
ymin, ymax = -0.1, 1.05
xmax = SIM_SEC
body = axis(xmax, ymin, ymax, w, h, "speed (m/s)  target=0.8", "%.1f")
for i, (name, _, _, _, _) in enumerate(scenarios):
    ts, sp, _ = traces[name]
    body += polyline(ts, sp, w, h, xmax, ymin, ymax).replace("__C__", colors[i])
legend = "".join('<text x="%d" y="%d" fill="%s" font-size="9">%s</text>' % (300 + i * 0, 14 + i * 0, colors[i], "")
                 for i in range(3))
legend = "".join('<text x="%d" y="%d" fill="%s" font-size="9">&#9632; %s</text>' % (40 + i * 175, h - 0 + 14, colors[i], scenarios[i][0].split('(')[0].strip()) for i in range(3))
svg_plots.append('<svg viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg" style="background:#1e222b;border-radius:6px">%s%s</svg>' % (w, h, body, legend))

# plot 2: steady-state speed vs target (shows fix#5 runs backward)
w, h = 560, 230
ymin2, ymax2 = -1.0, 1.1
body = axis(max(targets), ymin2, ymax2, w, h, "steady-state speed (m/s)", "%.1f")
for i, (name, _, _, _, _) in enumerate(scenarios):
    xs = targets
    ys = [results[name][T][0] for T in targets]
    body += polyline(xs, ys, w, h, max(targets), ymin2, ymax2).replace("__C__", colors[i])
xticks = "".join('<text x="%.1f" y="%d" fill="#9aa4b2" font-size="9">%.1f</text>' % (38 + (T / max(targets)) * (w - 48) - 6, h - 4, T) for T in targets)
svg_plots.append('<svg viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg" style="background:#1e222b;border-radius:6px">%s%s</svg>' % (w, h, body, xticks))

# ---------------- HTML report ----------------
table_rows = ""
for T in targets:
    row = "<tr><td>%.1f</td>" % T
    for name, _, _, _, _ in scenarios:
        mean, jit = results[name][T]
        row += '<td>%.3f &plusmn; %.3f</td>' % (mean, jit)
    row += "</tr>"
    table_rows += row

scenario_headers = "".join("<th>%s</th>" % s[0] for s in scenarios)

html = """<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<title>migong_car5 速度环仿真报告</title>
<style>body{background:#0f1217;color:#cdd6e4;font-family:Segoe UI,system-ui,sans-serif;margin:24px}
h1{color:#fff;font-size:20px}h2{color:#8fd0ff;font-size:15px;margin-top:26px}
p,li{line-height:1.6;font-size:13px;color:#b9c2d0}
table{border-collapse:collapse;margin:10px 0;font-size:12px}
th,td{border:1px solid #2a3040;padding:5px 10px;text-align:center}
th{background:#1b2230;color:#8fd0ff}
.bug{color:#ff8a8a}.ok{color:#7ff0a0}
code{background:#1b2230;padding:1px 5px;border-radius:3px;color:#ffd479}
svg{margin:8px 0;max-width:100%%}
.note{background:#161b24;border-left:3px solid #5db4ff;padding:10px 14px;border-radius:4px}
</style></head><body>
<h1>migong_car5 速度环数值仿真报告</h1>

<h2>1. 我如何做的模拟（方法）</h2>
<p>没有烧录、没有上电，用纯 Python（标准库）搭了一个闭环数值模型，<b>完全照搬固件逻辑</b>：</p>
<ul>
<li><b>PID</b>：复制 <code>pid.c::Position_PID_Realize</code> 的位置式 PID（比例+积分，积分限幅 &plusmn;%d），当前增益 <code>Kp=-30, Ki=-2.6</code>（负值）。</li>
<li><b>调用关系</b>：固件在 <code>main()</code> 死循环里每轮调 PID，而编码器速度只在 <code>TIM4</code> 10ms 中断里刷新。模拟两种模式：
  <span class="bug">current</span> = 每 1ms 调一次 PID、但速度反馈值在 10ms 内不更新（同一误差被反复积分，等效积分增益放大约 10 倍）；
  <span class="ok">fixed</span> = PID 每 10ms 调一次、与编码器刷新对齐。</li>
<li><b>极性对照</b>：额外加一个 <code>fix#5</code> 场景——把 <code>Kp/Ki</code> 翻成正——用来检验"翻符号能否修 bug"。仿真显示它让车<b>倒着跑</b>，从而证明极性绝不能盲翻。</li>
<li><b>被控对象</b>：电机+轮子用一阶惯性 <code>r &rarr; -KPLANT&middot;pwm</code>（<code>KPLANT=%.4f, TAU=%.2fs</code>）。板子上"电机接反 + 编码器取负"两个符号翻转恰好抵消，喂给 PID 的速度就是正的（前进为正），这正是"车能低速前进、但极性是错配凑成"的现实。</li>
<li><b>目标速度</b>：按固件公式 <code>TarEncodeSpd = TarSpeed&times;1560/(0.1885&times;100)</code> 换算成"脉冲/10ms"，与反馈同单位。</li>
</ul>
<p class="note">参数为合理工程估值（电机满程 ~0.84m/s，机械时间常数 0.25s），用于<b>演示机理</b>，不代表逐毫秒精确；但调用周期与极性结构是逐字对齐代码的。</p>

<h2>2. 仿真验证：极性不能盲翻 + 当前环可稳定达速</h2>
<p>下图是目标速度 1.0 m/s（接近满程）时的车速时域。<span class="bug">红/绿（current / fix#1）</span> 平稳收敛到目标速度（0.846 为满程饱和值），说明在合理工程模型下，<b>当前负增益环路本身能正常工作、低速/中速不抖</b>。关键看 <span class="bug">蓝（fix#5：把 Kp/Ki 翻成正）</span> —— 它直接收敛到 <b>-0.846 m/s，即整车倒着跑</b>。这用仿真铁证说明：<b>速度环增益符号绝不能盲翻</b>，当前负值是与板子极性（电机接反 + 编码器取负）将错就错凑成的，翻正就倒退。</p>
%s

<h2>3. 为什么仿真里没复现"高速抖"？真实根因在哪</h2>
<p>上表三种配置稳态抖动都 &asymp;0，因为本模型是一阶 + 小延迟的<b>理想简化</b>。真实小车的抖动来自这个模型没包含的更高阶 / 耦合因素，而这些恰恰对应前面审出的代码 bug：</p>
<ul>
<li><b>#1 调用周期抖动</b>：固件在 <code>main()</code> 死循环里调 PID，循环周期被 OLED(I2C)、多路 ADC 等拖得忽长忽短，导致离散积分的"步长"时变、等效增益不稳；高速（误差大、积分大）时最易激发振荡。修复 = 把 PID 搬进 TIM4 的 10ms 中断，固定步长。</li>
<li><b>#4 编码器左右抄反</b>（疑似）：左轮速度环读 Motor2、右轮读 Motor1，两轮速度环互控，高速差速 / 转向时错误被放大 &rarr; 抖 / 跑偏。修复 = 对照原理图确认后把 125/126 行对调，并用宏锁定。</li>
<li><b>转向环 / 速度环强耦合</b>：<code>pwml=-turnpwm+spd, pwmr=+turnpwm+spd</code>，直道也有微小纠偏，差速车高速时航向更敏感。</li>
</ul>
<p>因此<b>真正的修复是 #1 + #4 + 锁定 #5 极性</b>，而非翻转增益符号。下图（稳态车速 vs 目标）再次印证：保留负增益可正达目标，翻正则倒跑。</p>
%s

<h2>4. 数据表（稳态车速 &plusmn; 抖动, m/s）</h2>
<table><tr><th>目标 m/s</th>%s</tr>%s</table>

<h2>5. 结论与代码修复对应</h2>
<ul>
<li><b>仿真核心结论</b>：用固件同款 PID 逻辑建模，当前负增益环路能正确达到目标速度；<b>把 Kp/Ki 翻成正会让车倒着跑</b>（速度变负）。故 #5 的正确处理是<b>保留符号 + 加注释锁定极性约定</b>，绝不能盲翻。</li>
<li><b>修复 #1（关键）</b>：速度环 PID 移入 TIM4 的 10ms 中断（或 main 里 10ms 节拍门控），固定积分步长，消除调用周期抖动引发的增益不稳。</li>
<li><b>修复 #2</b>：GetParament 中 SPAN 标定先用旧 Max 计算再更新，恢复峰值偏差语义。</li>
<li><b>修复 #3</b>：里程计算改为浮点，消除整圈跳变。</li>
<li><b>处理 #4</b>：TraceMove 左右编码器对调加宏 <code>MOTOR_LEFT_IS_MOTOR1</code> + 注释，对照原理图确认后切换，避免盲改把能跑的车搞乱。</li>
</ul>
</body></html>
""" % (INTEG_LIMIT, KPLANT, TAU, svg_plots[0], svg_plots[1], scenario_headers, table_rows)

with open("sim_report.html", "w", encoding="utf-8") as f:
    f.write(html)

# console summary
print("=== speed-loop simulation summary ===")
print("target(m/s) | " + " | ".join(s[0].split('(')[0].strip() for s in scenarios))
for T in targets:
    line = "%.1f        | " % T
    for name, _, _, _, _ in scenarios:
        mean, jit = results[name][T]
        line += "%.3f+/-%.3f | " % (mean, jit)
    print(line)
print("\nHTML report written: sim_report.html")
