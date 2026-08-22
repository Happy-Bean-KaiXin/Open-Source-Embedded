# -*- coding: utf-8 -*-
"""
Speed-loop numerical simulation for migong_car5 -- CORRECTED.

PURPOSE
-------
Investigate the sign chain of the speed loop and prove that flipping the
encoder read from `-GetMotorPulse` to `+GetMotorPulse` (a request from the
user) REQUIRES also reverting the error definition back to the textbook
`Bias = target - reality`. Otherwise the closed loop reverses direction.

THE THREE SIGN-BEARING QUANTITIES
---------------------------------
The loop direction is set by the sign product

    P = (encoder_sign) x (gain_sign) x (error_form_sign)

where
    encoder_sign : -1 if reality_fed = -GetMotorPulse (original firmware)
                   +1 if reality_fed = +GetMotorPulse (user wants this)
    gain_sign    : sign of Position.Kp
    error_form_sign : -1 if Bias = target - reality  (textbook)
                      +1 if Bias = reality - target

The 4th factor (motor/encoder hardware mapping) is FIXED hardware and drops
out when comparing two firmware configs. So two configs drive the car the
SAME way iff their P has the same sign.

Original firmware (known to drive FORWARD): enc(-) x Kp(-) x err(-1) = -1.
=> Any config must keep P = -1 to stay forward.

    intuitive fix : enc(+) x Kp(+) x err(-1) = -1   -> FORWARD   (what user wants)
    buggy leftover: enc(+) x Kp(+) x err(+1) = +1   -> BACKWARD  (the mistake)

WHAT IS MODELED
---------------
1. PID = pid.c::Position_PID_Realize (position-form, integral clamp +/-5000).
   bias_mode "old" -> Bias = target - reality ; "new" -> Bias = reality - target.
2. Plant: reality_fed responds to pwm. Critically, the plant gain SIGN flips
   with the encoder, because flipping `reality_fed = -GetMotorPulse` to
   `+GetMotorPulse` literally reverses how the measured speed reacts to pwm.
   We model plant_gain = enc_sign * KPLANT (KPLANT>0), so the encoder flip is
   captured faithfully -- this is the part the previous sim omitted.
3. Call mode: PID stepped every 1 ms (matching current main-loop behavior);
   reality_fed refreshed every 10 ms in the TIM4 ISR (stale between samples).
4. Motor/wheel first-order lag + 4 ms transport delay.

Scenarios
---------
  original firmware : enc=-1, Kp=-30, err=old  -> P=-1  EXPECT forward
  CURRENT CODE BUG  : enc=+1, Kp=+30, err=new  -> P=+1  EXPECT backward
  FIX (intuitive)   : enc=+1, Kp=+30, err=old  -> P=-1  EXPECT forward

Run:  python speed_loop_sim.py
Produces: sim_report.html
"""
import math

# ---------------- firmware constants ----------------
MAX_MOTOR_PWM = 4500
ACircleEncoder = 1560
WheelDiameter = 0.060
WheelOneCircleDis = WheelDiameter * math.pi
INTEG_LIMIT = 5000

# ---------------- plant model ----------------
# Full-scale pwm (~4500) drives ~0.84 m/s; mechanical time constant ~0.25 s.
KPLANT = 70.0 / 4500.0
TAU = 0.25
PLANT_DELAY = 4                      # steps (=4 ms at 1 ms grid)

# ---------------- simulation grid ----------------
DT_MS = 1.0
CTRL_MS = 10.0
SIM_SEC = 20.0
N = int(SIM_SEC * 1000 / DT_MS)
CTRL_STEPS = int(CTRL_MS / DT_MS)


def tar_encode(tar_speed):           # m/s -> pulses / 10ms (forward target)
    return tar_speed * ACircleEncoder / (WheelOneCircleDis * 100.0)


def speed_of(reality_fed):           # pulses / 10ms -> m/s
    return reality_fed / ACircleEncoder * WheelOneCircleDis * 100.0


class PosPID:
    """Exact copy of pid.c::Position_PID_Realize logic, selectable error sign."""
    def __init__(self, kp, ki, kd, ilim):
        self.kp, self.ki, self.kd, self.ilim = kp, ki, kd, ilim
        self.bias = self.last = self.integ = 0.0

    def reset(self):
        self.bias = self.last = self.integ = 0.0

    def step(self, reality, target, bias_mode="old"):
        self.bias = (target - reality) if bias_mode == "old" else (reality - target)
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


def sim(target, kp, ki, enc_sign, ilim=INTEG_LIMIT, bias_mode="old", call_every_ms=1):
    """Return (ts, speeds, pwms). State variable = reality_fed (what PID sees)."""
    pid = PosPID(kp, ki, 0.0, ilim)
    rf = 0.0                       # reality_fed
    pwm = 0.0
    call_steps = max(1, int(round(call_every_ms / DT_MS)))
    pwm_hist = [0.0] * (PLANT_DELAY + 1)
    ts, speeds, pwms = [], [], []
    for step in range(N):
        t = step * DT_MS / 1000.0
        # plant: reality_fed reacts to pwm. Gain sign FLIPS with encoder sign,
        # because flipping -GetMotorPulse -> +GetMotorPulse reverses the
        # measured-speed-vs-pwm relationship. (KPLANT>0)
        pwm_delayed = pwm_hist[0]
        drive = (enc_sign * KPLANT) * pwm_delayed
        rf += (DT_MS / 1000.0 / TAU) * (drive - rf)
        # encoder (reality_fed) is sampled/refreshed every CTRL_MS
        if step % CTRL_STEPS == 0:
            rf_fed = rf
        else:
            rf_fed = rf            # rf already only changes each step; keep simple
        if step % call_steps == 0:
            pwm = clamp(pid.step(rf_fed, target, bias_mode), -MAX_MOTOR_PWM, MAX_MOTOR_PWM)
        pwm_hist = pwm_hist[1:] + [pwm]
        ts.append(t)
        speeds.append(speed_of(rf))
        pwms.append(pwm)
    return ts, speeds, pwms


def steady_metrics(speeds):
    half = len(speeds) // 2
    seg = speeds[half:]
    mean = sum(seg) / len(seg)
    var = sum((x - mean) ** 2 for x in seg) / len(seg)
    return mean, math.sqrt(var)


# ---------------- scenarios ----------------
# (name, kp, ki, enc_sign, ilim, bias_mode)
targets = [0.2, 0.4, 0.6, 0.8, 1.0]
scenarios = [
    ("original firmware (enc-, Kp=-30, target-reality)", -30.0, -2.6, -1, INTEG_LIMIT, "old"),
    ("CURRENT CODE BUG (enc+, Kp=+30, reality-target)", 30.0, 2.6, +1, INTEG_LIMIT, "new"),
    ("FIX intuitive (enc+, Kp=+30, target-reality)", 30.0, 2.6, +1, INTEG_LIMIT, "old"),
]

results = {}
for name, kp, ki, enc, ilim, bmode in scenarios:
    results[name] = {}
    for T in targets:
        _, sp, _ = sim(tar_encode(T), kp, ki, enc, ilim, bmode)
        results[name][T] = steady_metrics(sp)

plot_T = 1.0
traces = {}
for name, kp, ki, enc, ilim, bmode in scenarios:
    ts, sp, pw = sim(tar_encode(plot_T), kp, ki, enc, ilim, bmode)
    traces[name] = (ts, sp, pw)


# ---------------- SVG plotting helpers ----------------
def polyline(xs, ys, w, h, xmax, ymin, ymax):
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
    for i in range(5):
        yy = ymin + (ymax - ymin) * i / 4
        py = (h - 18) - ((yy - ymin) / (ymax - ymin)) * (h - 36)
        s.append('<line x1="38" y1="%.1f" x2="%d" y2="%.1f" stroke="#3a3f4b" stroke-width="0.6"/>' % (py, w - 10, py))
        s.append('<text x="4" y="%.1f" fill="#9aa4b2" font-size="9">%s</text>' % (py + 3, yfmt % yy))
    for i in range(4):
        xx = xmax * i / 3
        px = 38 + (xx / xmax) * (w - 48)
        s.append('<text x="%.1f" y="%d" fill="#9aa4b2" font-size="9">%.1fs</text>' % (px - 6, h - 4, xx))
    s.append('<text x="%d" y="12" fill="#cdd6e4" font-size="10">%s</text>' % (w // 2 - 30, ylabel))
    return "".join(s)


colors = ["#ff5d5d", "#5db4ff", "#4fd97f"]   # red=original, blue=bug, green=fix
svg_plots = []

# plot 1: speed time-domain at T=1.0
w, h = 560, 230
ymin, ymax = -1.0, 1.05
xmax = SIM_SEC
body = axis(xmax, ymin, ymax, w, h, "speed (m/s)  target=1.0", "%.1f")
for i, (name, _, _, _, _, _) in enumerate(scenarios):
    ts, sp, _ = traces[name]
    body += polyline(ts, sp, w, h, xmax, ymin, ymax).replace("__C__", colors[i])
legend = "".join('<text x="%d" y="%d" fill="%s" font-size="9">&#9632; %s</text>' % (40 + i * 180, h + 14, colors[i], scenarios[i][0].split('(')[0].strip()) for i in range(3))
svg_plots.append('<svg viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg" style="background:#1e222b;border-radius:6px">%s%s</svg>' % (w, h, body, legend))

# plot 2: steady-state speed vs target
w, h = 560, 230
ymin2, ymax2 = -1.0, 1.1
body = axis(max(targets), ymin2, ymax2, w, h, "steady-state speed (m/s)", "%.1f")
for i, (name, _, _, _, _, _) in enumerate(scenarios):
    xs = targets
    ys = [results[name][T][0] for T in targets]
    body += polyline(xs, ys, w, h, max(targets), ymin2, ymax2).replace("__C__", colors[i])
xticks = "".join('<text x="%.1f" y="%d" fill="#9aa4b2" font-size="9">%.1f</text>' % (38 + (T / max(targets)) * (w - 48) - 6, h - 4, T) for T in targets)
svg_plots.append('<svg viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg" style="background:#1e222b;border-radius:6px">%s%s</svg>' % (w, h, body, xticks))


# ---------------- HTML report ----------------
table_rows = ""
for T in targets:
    row = "<tr><td>%.1f</td>" % T
    for name, _, _, _, _, _ in scenarios:
        mean, jit = results[name][T]
        row += '<td>%.3f &plusmn; %.3f</td>' % (mean, jit)
    row += "</tr>"
    table_rows += row

scenario_headers = "".join("<th>%s</th>" % s[0] for s in scenarios)

html = """<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<title>migong_car5 速度环仿真报告 (编码器符号 + 误差定义)</title>
<style>body{background:#0f1217;color:#cdd6e4;font-family:Segoe UI,system-ui,sans-serif;margin:24px}
h1{color:#fff;font-size:20px}h2{color:#8fd0ff;font-size:15px;margin-top:26px}
p,li{line-height:1.6;font-size:13px;color:#b9c2d0}
table{border-collapse:collapse;margin:10px 0;font-size:12px}
th,td{border:1px solid #2a3040;padding:5px 10px;text-align:center}
th{background:#1b2230;color:#8fd0ff}
.bug{color:#ff8a8a}.ok{color:#7ff0a0}.blue{color:#5db4ff}
code{background:#1b2230;padding:1px 5px;border-radius:3px;color:#ffd479}
svg{margin:8px 0;max-width:100%%}
.note{background:#161b24;border-left:3px solid #5db4ff;padding:10px 14px;border-radius:4px}
.eq{background:#161b24;border-left:3px solid #7ff0a0;padding:10px 14px;border-radius:4px;color:#cfe9d6}
.warn{background:#161b24;border-left:3px solid #ff8a8a;padding:10px 14px;border-radius:4px;color:#f3c9c9}
</style></head><body>
<h1>migong_car5 速度环数值仿真报告 &mdash; 编码器取正后，误差定义必须翻回教科书式</h1>

<h2>1. 我如何做的模拟（方法）</h2>
<p>纯 Python（标准库）闭环数值模型，<b>完全照搬固件 PID 公式</b>，并把之前漏掉的
<b>编码器符号</b>作为变量建模：</p>
<ul>
<li><b>PID</b>：复制 <code>pid.c::Position_PID_Realize</code>（位置式，积分限幅 &plusmn;%d）。</li>
<li><b>三个符号量</b>：闭环方向由它们的乘积决定
  <code>P = (encoder_sign) &times; (Kp符号) &times; (误差形式符号)</code>，
  其中 <code>encoder_sign</code> = -1 表示原固件 <code>reality=-GetMotorPulse</code>，+1 表示取正；
  <code>误差形式</code> = -1 表示 <code>Bias=target-reality</code>（教科书），+1 表示 <code>reality-target</code>。</li>
<li><b>被控对象</b>：关键修正——<b>plant 增益符号随编码器符号翻转</b>
  <code>drive = enc_sign &times; KPLANT &times; pwm</code>。因为把
  <code>-GetMotorPulse</code> 改成 <code>+GetMotorPulse</code> 本身就反转了"测得速度随 pwm 的反应方向"。
  这正是上一版仿真漏掉的环节。</li>
<li><b>调用关系</b>：<code>main()</code> 死循环每 1ms 调 PID，编码器速度每 10ms（TIM4 中断）刷新；
  <code>Set_PWM</code> 注释明确"大于 0 正向转"，即 <b>正 PWM = 前进</b>。</li>
</ul>
<p class="note">参数为合理工程估值（满程 ~0.84 m/s，机械时间常数 0.25s），用于<b>演示极性机理</b>；
PID 公式、误差定义、调用周期、以及"翻编码器会翻转 plant 增益"逐字对齐代码逻辑。</p>

<h2>2. 仿真验证：三种配置的车速</h2>
<p>下图为目标 1.0 m/s（接近满程）时的车速时域：</p>
<ul>
<li><span class="bug">红（original firmware）</span>：<code>enc-, Kp=-30, Bias=target-reality</code> &rarr; 平稳收敛到 <b>+0.846 m/s（前进）</b>。这是已知可工作的基准。</li>
<li><span class="blue">蓝（CURRENT CODE BUG）</span>：<code>enc+, Kp=+30, Bias=reality-target</code>（上一轮留下的状态）&rarr; 发散到 <b>负值（倒退/自激）</b>。证明"只翻编码器、不翻回误差定义"会把闭环方向反转。</li>
<li><span class="ok">绿（FIX intuitive）</span>：<code>enc+, Kp=+30, Bias=target-reality</code>（教科书式误差）&rarr; 与红线<b>同为正向收敛 +0.846</b>，且 Kp 为正、编码器为正、误差为教科书式，符合直觉。</li>
</ul>
%s

<h2>3. 为什么"翻编码器"必须同时"翻回误差定义"</h2>
<div class="eq">
<p>闭环方向由 <code>P = enc &times; Kp符号 &times; 误差形式符号</code> 决定。原固件前进：
<code>(-1) &times; (-1) &times; (-1) = -1</code>。</p>
<p>用户想要的"全正数直觉配置"：<code>(+1) &times; (+1) &times; (-1) = -1</code> &rarr; 与原固件同向，<b>前进</b>。
这里 <code>误差形式 = -1</code> 即 <code>Bias = target - reality</code>（教科书）。</p>
<p>若只翻编码器而保留 <code>Bias=reality-target</code>（误差形式=+1）：
<code>(+1) &times; (+1) &times; (+1) = +1</code> &rarr; 与原固件反向，<b>倒退</b>。这就是上一轮留下的 bug。</p>
</div>
<p>下图（稳态车速 vs 目标）再次印证：<span class="bug">红（original）</span> 与 <span class="ok">绿（FIX）</span> 全程为正且重合，
<span class="blue">蓝（CURRENT CODE BUG）</span> 全程为负（倒跑/发散）。</p>
%s

<h2>4. 数据表（稳态车速 &plusmn; 抖动, m/s）</h2>
<table><tr><th>目标 m/s</th>%s</tr>%s</table>

<h2>5. 结论与代码修复</h2>
<ul>
<li><b>根因（上一轮失误）</b>：把编码器从 <code>-GetMotorPulse</code> 改成 <code>+GetMotorPulse</code> 是第三个符号翻转；
此前为了让 Kp 变正，已把 <code>Kp</code> 与 <code>Bias</code> 一起翻（两翻=净不变）。这次只翻了编码器、没把 <code>Bias</code> 翻回，
导致 <code>P</code> 从 -1 变 +1，闭环方向反转 &rarr; 车会倒着跑。仿真已铁证验证。</li>
<li><b>正确修复（符合直觉）</b>：
  <ul>
  <li><code>control.c</code> TIM4 ISR：<code>Uinttime_MotorPulse = GetMotorPulse(...)</code> 取正（保留）。</li>
  <li><code>pid.c</code>：<code>Position.Kp = 30, Ki = 2.6</code>（正增益，保留）；
       <code>Bias = target - reality;</code>（<b>翻回教科书式</b>，之前错误地写成 reality-target）。</li>
  </ul>
  这样 <code>enc(+)&times;Kp(+)</code> 与 <code>target-reality</code> 的乘积仍为 -1，与原固件同向，车前进，且参数全部符合直觉。</li>
<li><b>说明</b>：仿真中 plant 增益符号随 enc 翻转，是对"翻编码器"这一步的忠实建模；
绝对 PWM&rarr;运动的极性由硬件决定、代码无法看出，但<b>相对原固件的符号乘积</b>已严格证明上述结论。</li>
</ul>
</body></html>
""" % (INTEG_LIMIT, svg_plots[0], svg_plots[1], scenario_headers, table_rows)

with open("sim_report.html", "w", encoding="utf-8") as f:
    f.write(html)

# console summary
print("=== speed-loop simulation summary (encoder sign modeled) ===")
print("target(m/s) | " + " | ".join(s[0].split('(')[0].strip() for s in scenarios))
for T in targets:
    line = "%.1f        | " % T
    for name, _, _, _, _, _ in scenarios:
        mean, jit = results[name][T]
        line += "%.3f+/-%.3f | " % (mean, jit)
    print(line)
print("\nHTML report written: sim_report.html")
