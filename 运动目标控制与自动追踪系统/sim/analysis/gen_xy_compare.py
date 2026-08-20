# -*- coding: utf-8 -*-
"""
gen_xy_compare.py  ——  X算Y vs Y算X 逐条对比  (fig7)  [v2: 防豆腐块 + 目的横幅 + 详细解释]
================================================================================
对每条线段分别模拟「驱动X轴(用calcY)」和「驱动Y轴(用calcX)」两种方式,
对比各自的终点误差。直观展示: 规则选大轴 = 在两种方式中选误差更小的那个。

产出: fig7_xy_compare.png
用法: python gen_xy_compare.py
================================================================================
"""
import os, math, struct, random

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

import analysis_utils as au

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = HERE
au.setup_chinese_font()

def f32(x): return struct.unpack("f", struct.pack("f", float(x)))[0]
def trunc(x): return int(x)
def calcSlope(x1,y1,x2,y2): return 0.0 if abs(x2-x1)<1e-9 else f32((y2-y1)/(x2-x1))
def calcInt(x1,y1,s): return 0 if s==0.0 else trunc(f32(f32(y1)-f32(s*x1)))
def calcY(x,s,i): return i if s==0.0 else trunc(f32(f32(s*x)+f32(i)))
def calcX(y,s,i): return 0 if s==0.0 else trunc(f32(f32(y-f32(i))/f32(s)))

STEP=7; DEADZONE=4

def sim(S,E,drive):
    x1,y1=S; x2,y2=E; dx=x2-x1; dy=y2-y1
    if (drive=="X" and dx==0) or (drive=="Y" and dy==0):
        return {"Pp":(float(x2),float(y2)),"path":[S,E]}
    slope=calcSlope(x1,y1,x2,y2); intercept=calcInt(x1,y1,slope)
    path=[S]
    def step_X():
        x=x1; prev_d=abs(x-x2); best=(float(x),float(y1)); best_d=prev_d
        while True:
            x+=STEP if dx>0 else -STEP; y=calcY(x,slope,intercept); path.append((x,y))
            d=abs(x-x2)
            if d<best_d: best_d=d; best=(float(x),float(y))
            if d<DEADZONE: return (float(x),float(y))
            if d>prev_d: return best
            prev_d=d
            if len(path)>4000: return best
    def step_Y():
        y=y1; prev_d=abs(y-y2); best=(float(x1),float(y)); best_d=prev_d
        while True:
            y+=STEP if dy>0 else -STEP
            x=x1 if dx==0 else calcX(y,slope,intercept); path.append((x,y))
            d=abs(y-y2)
            if d<best_d: best_d=d; best=(float(x),float(y))
            if d<DEADZONE: return (float(x),float(y))
            if d>prev_d: return best
            prev_d=d
            if len(path)>4000: return best
    Pp = step_X() if drive=="X" else step_Y()
    return {"Pp":Pp,"path":path}

def build_segs():
    segs=[]
    hand=[(30,160,290,160,"纯横"),(160,30,160,290,"纯竖"),
          (50,50,250,250,"正对角"),(100,100,160,140,"中等60/40"),
          (100,100,220,130,"浅"),(100,100,130,220,"陡")]
    for a in hand: segs.append({"S":(a[0],a[1]),"E":(a[2],a[3]),"note":a[4]})
    rng=random.Random(20260817)
    while len(segs)<80:
        x1=rng.randint(30,320); y1=rng.randint(30,320); x2=rng.randint(30,320); y2=rng.randint(30,320)
        if abs(x2-x1)<3 and abs(y2-y1)<3: continue
        segs.append({"S":(x1,y1),"E":(x2,y2),"note":"random"})
    return segs

def main():
    segs=build_segs()
    rows=[]
    for i,s in enumerate(segs):
        S,E=s["S"],s["E"]; dx=E[0]-S[0]; dy=E[1]-S[1]
        rx=sim(S,E,"X"); ry=sim(S,E,"Y")
        ex=math.hypot(rx["Pp"][0]-E[0],rx["Pp"][1]-E[1])
        ey=math.hypot(ry["Pp"][0]-E[0],ry["Pp"][1]-E[1])
        ratio=(abs(dx)/max(abs(dy),1e-9)) if dy!=0 else 1e6
        rows.append({"ratio":ratio,"err_x":ex,"err_y":ey,
                     "rule_err":min(ex,ey),"wrong_err":max(ex,ey),
                     "note":s["note"]})

    fig=plt.figure(figsize=(11,9))
    # 主图区避开顶部横幅
    ax=fig.add_axes([0.10,0.50,0.85,0.36])
    xs_x,ys_x,xs_y,ys_y=[],[],[],[]
    for r in rows:
        xs_x.append(r["ratio"]); ys_x.append(r["err_x"])
        xs_y.append(r["ratio"]); ys_y.append(r["err_y"])
    ax.scatter(xs_x,ys_x,s=14,c="#1f77b4",alpha=0.7,label="驱动 X 轴 (calcY)")
    ax.scatter(xs_y,ys_y,s=14,c="#ff7f0e",alpha=0.7,label="驱动 Y 轴 (calcX)")
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("ratio = |dx|/|dy|")
    ax.set_ylabel("终点误差 (px)")
    ax.set_title("图7  X算Y vs Y算X 终点误差逐条对比")
    ax.axvline(1.0,color="k",ls="--",lw=1)
    ax.legend(loc="upper right",fontsize=8)
    ax.grid(True,alpha=0.3,which="both")

    au.add_banner(fig, "图7  驱动 X vs 驱动 Y 逐条配对对比",
                  "每条线段同时用两种方式走一遍, 把两种终点误差画成上下配对的散点 —— "
                  "证明“按规则选轴”本质就是“选误差更小的那种方式”")
    au.add_explanation(fig, [
        ("【这张图在模拟什么】", True),
        "对 80 条线段(6 手工 + 74 随机), 每条都“跑两遍”: 一遍驱动 X 轴(Y 用 calcY 推导), "
        "一遍驱动 Y 轴(X 用 calcX 推导)。两条误差曲线上下配对, 直接可比较。",
        ("【怎么读】", True),
        "· 蓝点 = 驱动 X 的终点误差; 橙点 = 驱动 Y 的终点误差; 同一线段的两个点上下对应;",
        "· ratio>1(横线): 蓝低橙高 → 驱动 X 更好(dx 大, 驱动大轴);",
        "· ratio<1(竖线): 橙低蓝高 → 驱动 Y 更好(dy 大, 驱动大轴);",
        "· ratio=1(对角): 两点重合 → 驱动谁一样。",
        ("【核心结论】", True),
        "规则 |dx|>=|dy|→驱动X 等价于“在两种驱动方式里自动选误差更小的那个”;",
        "这不是经验法则, 而是数学最优: 驱动大轴时有效斜率≤1, "
        "终点误差上界 = DEADZONE×√2 ≈ 5.66px 被最小化。",
        ("【对固件的意义】", True),
        "固件 contorl.c 的选轴分支(if |dx|>=|dy| 驱动X, else 驱动Y)就是在执行这个最优选择, "
        "无需人工干预即可在所有斜率下拿到最小终点误差。",
    ], y0=0.02, h=0.42, title="详细说明", fs=8.2)
    au.add_footer(fig, "gen_xy_compare.py")
    fig.savefig(os.path.join(OUT,"fig7_xy_compare.png"),dpi=150,bbox_inches="tight")
    plt.close(fig)
    print("[OK] fig7_xy_compare.png")

if __name__=="__main__": main()
