# -*- coding: utf-8 -*-
"""
gen_schematic.py  ——  空间示意图 (fig6)  [v2: 防豆腐块 + 目的横幅 + 详细解释]
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
from matplotlib.patches import FancyBboxPatch, Rectangle

import analysis_utils as au

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = HERE
au.setup_chinese_font()          # 自动探测中文字体, 防豆腐块


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
    # 主图压到 y 0.30~0.90 (横幅占顶部 0.93~1.0)
    ax = fig.add_axes([0.08, 0.30, 0.84, 0.55])

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
    ax.text(S[0]-8, S[1]+5, "S(%d,%d)" % S, fontsize=10,
            ha="right", color="k")
    ax.text(E[0]+8, E[1]+5, "E(%d,%d)" % E, fontsize=10,
            ha="left", color="r")
    ax.text(Pp[0]+8, Pp[1]-8, "P'(%.1f,%.1f)\nerr=%.2fpx" % (Pp[0], Pp[1], err),
            fontsize=9, ha="left", color="#d62728")

    # ---- 放大窗 (inset) ----
    inset_ax = fig.add_axes([0.58, 0.33, 0.34, 0.22])
    inset_ax.set_title("放大: P' 与 E 的偏差", fontsize=9)
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

    ax.set_xlabel("X (px)")
    ax.set_ylabel("Y (px)")
    ax.set_title("图6  空间示意图: S(100,100)->E(160,140)->P'(%.1f,%.1f)  误差=%.2fpx"
                 % (Pp[0], Pp[1], err))
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")

    # ---- 目的横幅 + 解释区 (自动换行) ----
    au.add_banner(fig, "图6  单条线段空间示意图",
                  "用一条具体线段把“终点误差”从抽象数字变成看得见的几何图形: "
                  "停点 P' 为什么不在目标 E 上, 差了多少, 这个误差由什么造成")
    au.add_explanation(fig, [
        ("【这张图在模拟什么】", True),
        "用一条具体的中等斜率线段 S(100,100)→E(160,140)(dx=60, dy=40, ratio=1.5) "
        "演示舵机实际怎么走: 规则要求驱动 X 轴, 每步 +7px, Y 坐标每步用斜率公式算出。",
        ("【图元素对照】", True),
        "· 绿直线 = 理想路径(期望轨迹); 蓝阶梯 = 固件每步的实际落点序列;",
        "· 黑圆 = 起点 S; 红星 = 目标 E; 橙菱形 = 实际停点 P'(进入 4px 死区即停);",
        "· 红色双向箭头 = P' 到 E 的终点误差; 右上小窗 = P' 与 E 的局部放大图。",
        ("【关键数字】", True),
        "· 步长 STEP=7px ≈ 20mm; 死区 DEADZONE=4px ≈ 11mm(够近就停);",
        "· 本线段终点误差 err=%.2fpx ≈ %.1fmm —— 由 int 截断 + 死区机制共同造成;"
        % (err, err * 1000.0 / 352.0),
        "· 物理换算: 1px ≈ 2.84mm (PX_PER_M=352)。",
        ("【工程含义】", True),
        "%.1fmm 的终点偏差对 0.5m 矩形标定场景可忽略; 若做“精确打点”, 可在规划层补偿停点, "
        "或到达后加一次小步微调(把最后 4px 死区误差吃掉)。" % (err * 1000.0 / 352.0),
    ], y0=0.01, h=0.26, title="详细说明", fs=8.2, wrap_width=100)

    fig.savefig(os.path.join(OUT, "fig6_schematic.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[OK] fig6_schematic.png  (err=%.2fpx)" % err)


if __name__ == "__main__":
    main()
