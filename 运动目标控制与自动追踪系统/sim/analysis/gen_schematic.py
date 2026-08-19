# -*- coding: utf-8 -*-
"""
gen_schematic.py  ——  空间示意图 (fig6)
================================================================================
画一条具体线段 S->E 的实际运动轨迹, 展示:
  - 起点 S、理想终点 E、实际停点 P'
  - 理想直线 (绿)
  - 实际阶梯路径 (蓝)
  - 放大窗显示 P' 与 E 的微小偏差
  - 标注误差值

用法:  python gen_schematic.py
================================================================================
"""
import os
import math
import struct

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyBboxPatch, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = HERE
_FONT = FontProperties(fname=r"C:\Windows\Fonts\simhei.ttf")
plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False


def f32(x):
    return struct.unpack("f", struct.pack("f", float(x)))[0]


def trunc(x): return int(x)
def calcSlope(x1,y1,x2,y2):
    return 0.0 if abs(x2-x1)<1e-9 else f32((y2-y1)/(x2-x1))
def calcInt(x1,y1,s):
    if s==0.0: return 0
    return trunc(f32(f32(y1)-f32(s*x1)))
def calcY(x,s,i):
    if s==0.0: return i
    return trunc(f32(f32(s*x)+f32(i)))
def calcX(y,s,i):
    if s==0.0: return 0
    return trunc(f32(f32(y-f32(i))/f32(s)))


STEP = 7
DEADZONE = 4


def simulate(S, E, drive):
    x1,y1=S; x2,y2=E; dx=x2-x1; dy=y2-y1
    step=STEP
    if (drive=="X" and dx==0) or (drive=="Y" and dy==0):
        return {"Pp":(float(x2),float(y2)),"path":[S,E]}
    slope=calcSlope(x1,y1,x2,y2); intercept=calcInt(x1,y1,slope)
    path=[S]
    def step_X():
        x=x1; prev_d=abs(x-x2); best=(float(x),float(y1)); best_d=prev_d
        while True:
            x+=step if dx>0 else -step; y=calcY(x,slope,intercept); path.append((x,y))
            d=abs(x-x2)
            if d<best_d: best_d=d; best=(float(x),float(y))
            if d<DEADZONE: return (float(x),float(y))
            if d>prev_d: return best
            prev_d=d
            if len(path) > 4000:
                return best
    def step_Y():
        y=y1; prev_d=abs(y-y2); best=(float(x1),float(y)); best_d=prev_d
        while True:
            y+=step if dy>0 else -step
            x=x1 if dx==0 else calcX(y,slope,intercept); path.append((x,y))
            d=abs(y-y2)
            if d<best_d: best_d=d; best=(float(x),float(y))
            if d<DEADZONE: return (float(x),float(y))
            if d>prev_d: return best
            prev_d=d
            if len(path) > 4000:
                return best
    Pp = step_X() if drive=="X" else step_Y()
    return {"Pp":Pp,"path":path}


def main():
    S = (100, 100)
    E = (160, 140)   # dx=60, dy=40, ratio=1.5
    drive_rule = "X"  # |dx|>=|dy|

    r = simulate(S, E, drive_rule)
    Pp = r["Pp"]
    path = r["path"]
    err = math.hypot(Pp[0]-E[0], Pp[1]-E[1])

    fig = plt.figure(figsize=(11, 10))
    ax = fig.add_axes([0.08, 0.35, 0.84, 0.55])

    # ---- 主图: 轨迹 + 理想线 ----
    ax.plot([S[0], E[0]], [S[1], E[1]], "-", c="#2ca02c", lw=2,
            label="理想直线 S -> E")
    px = [p[0] for p in path]; py = [p[1] for p in path]
    ax.plot(px, py, "-o", c="#1f77b4", ms=5, lw=1.3, label="规则驱动路径 (每步+7px)")
    # 起点 / 目标 / 停点
    ax.scatter(*S, c="k", s=80, zorder=6, label="起点 S")
    ax.scatter(*E, c="r", s=100, zorder=6, marker="*", label="目标 E")
    ax.scatter(*Pp, c="#ff7f0e", s=120, zorder=6, marker="D",
               label="实际停点 P' (%.1f, %.1f)" % Pp)

    # 误差箭头
    ax.annotate("", xy=E, xytext=Pp,
                arrowprops=dict(arrowstyle="<->", color="#d62728", lw=2))

    # 文字标注
    ax.text(S[0]-8, S[1]+5, "S(%d,%d)" % S, fontproperties=_FONT, fontsize=10,
            ha="right", color="k")
    ax.text(E[0]+8, E[1]+5, "E(%d,%d)" % E, fontproperties=_FONT, fontsize=10,
            ha="left", color="r")
    ax.text(Pp[0]+8, Pp[1]-8, "P'(%.1f,%.1f)\nerr=%.2fpx" % (Pp[0], Pp[1], err),
            fontproperties=_FONT, fontsize=9, ha="left", color="#d62728")

    # ---- 放大窗 (inset) ----
    inset_ax = fig.add_axes([0.58, 0.38, 0.34, 0.22])
    inset_ax.set_title("放大: P' 与 E 的偏差", fontproperties=_FONT, fontsize=9)
    cx, cy = (E[0]+Pp[0])/2, (E[1]+Pp[1])/2
    span = max(abs(E[0]-Pp[0]), abs(E[1]-Pp[1])) * 2.5 + 15
    inset_ax.set_xlim(cx-span/2, cx+span/2)
    inset_ax.set_ylim(cy-span/2, cy+span/2)
    inset_ax.plot([S[0], E[0]], [S[1], E[1]], "-", c="#2ca02c", lw=1.5)
    inset_ax.plot(px, py, "-o", c="#1f77b4", ms=4, lw=1)
    inset_ax.scatter(*E, c="r", s=80, zorder=6, marker="*")
    inset_ax.scatter(*Pp, c="#ff7f0e", s=100, zorder=6, marker="D")
    inset_ax.grid(True, alpha=0.3)
    for spine in inset_ax.spines.values():
        spine.set_color("#888")

    ax.set_xlabel("X (px)", fontproperties=_FONT)
    ax.set_ylabel("Y (px)", fontproperties=_FONT)
    ax.set_title("图6  空间示意图: S(100,100)->E(160,140)->P'(%.1f,%.1f)  误差=%.2fpx"
                 % (Pp[0], Pp[1], err), fontproperties=_FONT)
    ax.legend(loc="upper left", prop=_FONT, fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")

    # ---- 解释区 ----
    exp_ax = fig.add_axes([0.02, 0.02, 0.96, 0.26])
    exp_ax.axis("off")
    border = FancyBboxPatch((0.005, 0.005), 0.99, 0.99,
                             boxstyle="square,pad=0",
                             transform=exp_ax.transAxes,
                             facecolor="#fafbfc", edgecolor="#999",
                             linewidth=1.2, zorder=1)
    exp_ax.add_patch(border)
    lines = [
        ("## 图6 在模拟什么", True),
        ("用一条具体的「中等斜率」线段(dx=60,dy=40)展示舵机实际运动轨迹。", False),
        ("绿色直线 = 理想路径; 蓝色阶梯 = 固件按规则驱动X轴的实际落点序列;", False),
        ("黑色圆点=起点S, 红色星号=目标E, 橙色菱形=实际停点P'(死区内停)。", False),
        ("## 关键数字", True),
        ("dx=60, dy=40, ratio=1.5 --> 规则要求驱动 X 轴。", False),
        ("步长 STEP=7, 死区 DEADZONE=4 --> 驱动轴进入目标+-4px 即停。", False),
        ("终点误差 = |P'E| = %.2f px (约 %.1f mm)。这是 int 截断 + 死区的综合结果。" % (
            err, err * 1000.0 / 352.0), False),
        ("## 物理含义", True),
        ("规划坐标单位为 px (像素), PX_PER_M=352 --> 1px ≈ 2.84mm。", False),
        ("STEP=7 ≈ 20mm (每次跳进约2cm), DEADZONE=4 ≈ 11mm (够近就停)。", False),
        ("%.2fpx 终点误差 ≈ %.1fmm -- 对标定矩形(0.5m边长)而言可忽略。" % (
            err, err * 1000.0 / 352.0), False),
    ]
    y = 0.96
    for txt, bold in lines:
        w = "bold" if bold else "normal"
        exp_ax.text(0.03, y, txt, transform=exp_ax.transAxes, va="top", ha="left",
                    fontsize=8.5, fontproperties=_FONT, color="#222", fontweight=w)
        y -= 0.075

    fig.savefig(os.path.join(OUT, "fig6_schematic.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[OK] fig6_schematic.png  (err=%.2fpx)" % err)


if __name__ == "__main__":
    main()
