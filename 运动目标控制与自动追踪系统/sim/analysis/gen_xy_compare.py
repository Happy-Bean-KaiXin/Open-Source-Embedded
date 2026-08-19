# -*- coding: utf-8 -*-
"""
gen_xy_compare.py  ——  X算Y vs Y算X 逐条对比
================================================================================
对每条线段分别模拟「驱动X轴(用calcY)」和「驱动Y轴(用calcX)」两种方式,
对比各自的终点误差。直观展示: 规则选大轴 = 在两种方式中选误差更小的那个。

产出: fig7_xy_compare.png
用法: python gen_xy_compare.py
================================================================================
"""
import os, csv, math, struct, random

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = HERE
_FONT = FontProperties(fname=r"C:\\Windows\\Fonts\\simhei.ttf")
plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

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
    ax=fig.add_axes([0.10,0.52,0.85,0.38])
    xs_x,ys_x,xs_y,ys_y=[],[],[],[]
    for r in rows:
        xs_x.append(r["ratio"]); ys_x.append(r["err_x"])
        xs_y.append(r["ratio"]); ys_y.append(r["err_y"])
    ax.scatter(xs_x,ys_x,s=14,c="#1f77b4",alpha=0.7,label="驱动 X 轴 (calcY)")
    ax.scatter(xs_y,ys_y,s=14,c="#ff7f0e",alpha=0.7,label="驱动 Y 轴 (calcX)")
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("ratio = |dx|/|dy|",fontproperties=_FONT)
    ax.set_ylabel("终点误差 (px)",fontproperties=_FONT)
    ax.set_title("图7  X算Y vs Y算X 终点误差逐条对比",fontproperties=_FONT)
    ax.axvline(1.0,color="k",ls="--",lw=1)
    ax.legend(loc="upper right",prop=_FONT,fontsize=8)
    ax.grid(True,alpha=0.3,which="both")

    # 解释区
    exp_ax=fig.add_axes([0,0.02,1,0.38]); exp_ax.axis("off")
    from matplotlib.patches import FancyBboxPatch
    exp_ax.add_patch(FancyBboxPatch((0.005,0.005),0.99,0.99,boxstyle="square,pad=0",
        transform=exp_ax.transAxes,facecolor="#fafbfc",edgecolor="#999",linewidth=1.2,zorder=1))
    lines=[
        ("## 这张图在模拟什么",True),
        ("每条线段都跑了两遍: 一遍驱动X(calcY推导y), 一遍驱动Y(calcX推导x)。",False),
        ("蓝点=驱动X的终点误差, 橙点=驱动Y的终点误差。同一条线的两个点上下对应。",False),
        ("## 核心规律",True),
        ("ratio>1 (横着): 蓝点低、橙点高 --> 驱动X更好 (dx大, 驱动大轴)",False),
        ("ratio<1 (竖着): 橙点低、蓝点高 --> 驱动Y更好 (dy大, 驱动大轴)",False),
        ("ratio=1 (对角): 两点重合 --> 驱动谁一样",False),
        ("## 结论",True),
        ("规则 |dx|>=|dy|->驱动X 等价于: 在两种方法中选误差更小的那个。",False),
        ("这不是经验法则,而是数学最优: 驱动大轴时有效斜率<=1, 误差上界最小化。",False),
    ]
    y=0.96
    for txt,bold in lines:
        exp_ax.text(0.03,y,txt,transform=exp_ax.transAxes,va="top",ha="left",
                    fontsize=8.5,fontproperties=_FONT,color="#222",fontweight=("bold" if bold else "normal"))
        y-=0.095
    fig.savefig(os.path.join(OUT,"fig7_xy_compare.png"),dpi=150,bbox_inches="tight")
    plt.close(fig)
    print("[OK] fig7_xy_compare.png")

if __name__=="__main__": main()
