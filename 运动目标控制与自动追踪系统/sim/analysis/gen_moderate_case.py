# -*- coding: utf-8 -*-
"""
中等差异区间仿真 v2：
  - 新增"理想状态线"（纯浮点、无int截断，仅有死区约束的理论下限）
  - 修正标注框（解释平局情况）
  - 新增"终点误差物理含义"示意图（实际停点P'到目标点E的距离）
严格复刻固件算术：float32 斜率 + (int) 向零截断；STEP=7，DEADZONE=4。
"""
import math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyBboxPatch

# 全局中文字体设置
plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False
# 矢量化文字：把所有文字转成SVG路径，彻底消灭字体缺失/豆腐块
plt.rcParams["svg.fonttype"] = "path"
plt.rcParams["pdf.fonttype"] = 3

# 显式加载 SimHei 字体（.ttf 单字体文件，比 .ttc 集合更稳定）
_font_simhei = FontProperties(fname=r"C:\Windows\Fonts\simhei.ttf")

F32 = np.float32
STEP = 7
DEADZONE = 4

def trunc(x):
    return int(math.trunc(float(x)))

def calc_slope(x1, y1, x2, y2):
    x1, y1, x2, y2 = map(F32, [x1, y1, x2, y2])
    if (x2 - x1) == F32(0.0):
        return F32(0.0)
    return F32((y2 - y1) / (x2 - x1))

def calc_intercept(x1, y1, slope):
    return trunc(y1 - float(slope) * x1)

def calc_y(x, slope, intercept):
    return trunc(float(slope) * x + intercept)

def calc_x(y, slope, intercept):
    y_f, intercept_f = F32(y), F32(intercept)
    return trunc(float((y_f - intercept_f) / slope))

# ---- 固件模式（有截断）----
def sim_path_firmware(x1, y1, x2, y2, driver):
    slope_f = calc_slope(x1, y1, x2, y2)
    intercept_f = calc_intercept(x1, y1, slope_f)
    pts = []
    if driver == "X":
        sx = 1 if x2 >= x1 else -1
        x = x1
        for _ in range(5000):
            y = calc_y(x, slope_f, intercept_f)
            pts.append((x, y))
            if abs(x - x2) < DEADZONE:
                break
            nx = x + sx * STEP
            if nx == x: break
            x = nx
    else:
        sy = 1 if y2 >= y1 else -1
        y = y1
        for _ in range(5000):
            if slope_f == F32(0.0):
                x = x1
            else:
                x = calc_x(y, slope_f, intercept_f)
            pts.append((x, y))
            if abs(y - y2) < DEADZONE:
                break
            ny = y + sy * STEP
            if ny == y: break
            y = ny
    return pts

# ---- 理想模式（纯浮点，无截断，仅死区约束）----
def sim_path_ideal(x1, y1, x2, y2, driver):
    """用 double 精度算直线参数，calc_y/calc_x 返回原始 float 不截断"""
    dx, dy = x2-x1, y2-y1
    if abs(dx) < 1e-12:
        slope = 0.0
    else:
        slope = dy / dx                # double 精度斜率
    intercept = y1 - slope * x1        # double 精度截距（不截断）
    pts = []
    if driver == "X":
        sx = 1 if x2 >= x1 else -1
        x = float(x1)
        for _ in range(5000):
            y = slope * x + intercept   # 纯浮点，不截断
            pts.append((x, y))
            if abs(x - x2) < DEADZONE:
                break
            nx = x + sx * STEP
            if nx == x: break
            x = nx
    else:
        sy = 1 if y2 >= y1 else -1
        y = float(y1)
        for _ in range(5000):
            if abs(slope) < 1e-12:
                x = float(x1)
            else:
                x = (y - intercept) / slope   # 纯浮点，不截断
            pts.append((x, y))
            if abs(y - y2) < DEADZONE:
                break
            ny = y + sy * STEP
            if ny == y: break
            y = ny
    return pts

def perp_dist(px, py, x1, y1, x2, y2):
    a = y1 - y2; b = x2 - x1; c = x1*y2 - x2*y1
    d = math.hypot(a, b)
    return 0.0 if d == 0 else abs(a*px + b*py + c) / d

def analyze_seg(x1, y1, x2, y2):
    dx, dy = x2-x1, y2-y1; adx, ady = abs(dx), abs(dy)
    rec = "X" if adx >= ady else "Y"
    out = {}
    for drv in ("X", "Y"):
        # 固件模式
        pts_fw = sim_path_firmware(x1, y1, x2, y2, drv)
        devs = [perp_dist(p[0], p[1], x1, y1, x2, y2) for p in pts_fw]
        fx, fy = pts_fw[-1]
        # 理想模式
        pts_id = sim_path_ideal(x1, y1, x2, y2, drv)
        idx, idy = pts_id[-1]
        out[drv] = dict(
            n=len(pts_fw),
            max_dev=max(devs),
            end_err=math.hypot(fx-x2, fy-y2),       # 固件终点误差
            end_err_ideal=math.hypot(idx-x2, idy-y2), # 理想终点误差
            end_pt=(fx, fy),                           # 固件实际停点
            end_pt_ideal=(idx, idy),                   # 理想停点
        )
    return dict(x1=x1, y1=y1, x2=x2, y2=y2, dx=dx, dy=dy, adx=adx, ady=ady,
                rec=rec, X=out["X"], Y=out["Y"])

def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))

    samples = [
        ("dx=60, dy=40 (浅)",  (100, 100, 160, 140)),
        ("dx=40, dy=60 (陡)",  (100, 100, 140, 160)),
        ("dx=50, dy=50 (对角)", (100, 100, 150, 150)),
        ("dx=64, dy=44 (浅)",  (100, 100, 164, 144)),
        ("dx=44, dy=64 (陡)",  (100, 100, 144, 164)),
    ]
    print("=== 中等差异区间 ===")
    print(f"{'线段':22s} {'规则':2s} {'固件误差':9s} {'理想误差':9s} {'截断贡献':9s} "
          f"{'X步':4s} {'Y步':4s} {'固件停点':18s} {'目标点':10s}")
    for name, seg in samples:
        r = analyze_seg(*seg)
        drv = r["rec"]
        fw_err = r[drv]["end_err"]
        id_err = r[drv]["end_err_ideal"]
        pt = r[drv]["end_pt"]
        print(f"{name:22s} {drv:2s} {fw_err:9.3f} {id_err:9.3f} {fw_err-id_err:9.3f} "
              f"{r['X']['n']:4d} {r['Y']['n']:4d} ({pt[0]:.1f},{pt[1]:.1f})     ({r['x2']},{r['y2']})")

    # ---------- 扫描曲线 ----------
    ratios = np.logspace(-1, 1, 200)
    rule_err, other_err = [], []
    rule_ideal, other_ideal = [], []   # 新增：理想状态线
    for ratio in ratios:
        dx = 40.0 * ratio; dy = 40.0
        x1, y1 = 100.0, 100.0
        x2, y2 = 100.0 + dx, 100.0 + dy
        r = analyze_seg(int(x1), int(y1), int(round(x2)), int(round(y2)))
        rec = r["rec"]
        rule_err.append(r[rec]["end_err"])
        other_err.append(r["Y"]["end_err"] if rec=="X" else r["X"]["end_err"])
        rule_ideal.append(r[rec]["end_err_ideal"])
        other_ideal.append(r["Y"]["end_err_ideal"] if rec=="X" else r["X"]["end_err_ideal"])

    # ---------- 主图（上半部分：曲线） ----------
    fig = plt.figure(figsize=(11, 26.0))
    ax = fig.add_axes([0.08, 0.58, 0.88, 0.36])  # 图表区压缩到上部36%

    # 三条线：固件规则 / 固件错选 / 理想状态
    ax.plot(ratios, rule_err, c="#1565c0", lw=2.2,
            label="固件-按规则选轴 (驱动大轴)")
    ax.plot(ratios, other_err, c="#ef6c00", lw=2.0, ls="--",
            label="固件-反着选轴 (驱动小轴)")
    ax.plot(ratios, rule_ideal, c="#2e7d32", lw=1.5, ls="-.",
            label="理想状态-按规则选轴 (纯浮点+仅死区约束)")

    # ratio=1 参考线 —— 标注放到图表右上角外部空白区，完全不被线遮挡
    ax.axvline(1.0, color="#444", lw=1.2, ls=":")
    ax.text(0.98, 0.98, "ratio=1\n(|dx|=|dy|\n对称/平局点)",
            va="top", ha="right", fontsize=8.5, color="#444",
            transform=ax.transAxes,
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#888", alpha=0.95),
            zorder=10)

    # 标注样本点 —— 标签用箭头指向外部空白区
    label_positions = []  # 收集标签位置避免重叠
    for name, seg in samples:
        r = analyze_seg(*seg)
        ratio = r["adx"] / max(r["ady"], 1e-9)
        err = r[r["rec"]]["end_err"]
        other = r["Y"]["end_err"] if r["rec"]=="X" else r["X"]["end_err"]
        ax.scatter([ratio], [err], c="#1565c0", s=40, zorder=5)
        ax.scatter([ratio], [other], c="#ef6c00", s=40, marker="^", zorder=5)
        # 标注文字放到点的上方/下方空白处（白底框+高zorder确保不被线遮挡）
        short = name.split()[0].replace("(","").replace(")","")  # 如 "dx=60,dy=40"
        offset_y = 12 if err < 10 else -14
        ax.annotate(short, (ratio, err),
                    textcoords="offset points", xytext=(0, offset_y),
                    fontsize=7, color="#1565c0", ha="center",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#1565c0", alpha=0.95),
                    zorder=10)

    ax.set_xscale("log")
    ax.set_xlabel("陡峭度 ratio = |dx| / |dy|   (<1 陡线应选Y , >1 浅线应选X)", fontsize=10)
    ax.set_ylabel("终点误差 (px) = 实际停点到目标点的直线距离", fontsize=10)
    ax.set_title("中等差异区间：终点误差对比（含理想状态下限）",
                 fontsize=13, fontweight="bold", pad=12)
    ax.legend(fontsize=9, loc="upper left")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, max(max(other_err), max(rule_err)) * 1.18)

    # 右上角摘要（修正平局标注）
    s60 = analyze_seg(100, 100, 160, 140)
    s40 = analyze_seg(100, 100, 140, 160)
    sd = analyze_seg(100, 100, 150, 150)
    summary = (
        "dx=60,dy=40: 规则(X) %.2fpx < 错选(Y) %.2fpx\n" % (s60["X"]["end_err"], s60["Y"]["end_err"]) +
        "dx=40,dy=60: 规则(Y) %.2fpx = 错选(X) %.2fpx (平局!)\n" % (s40["Y"]["end_err"], s40["X"]["end_err"]) +
        "           ^ 整数截断巧合导致两者碰巧相等\n" +
        "对角线:      两轴均 %.2fpx (对称平局)" % sd["X"]["end_err"]
    )
    ax.text(0.98, 0.65, summary, transform=ax.transAxes, ha="right", va="top",
            fontsize=8.5, color="#333",
            bbox=dict(boxstyle="round,pad=0.4", fc="#e3f2fd", ec="#1565c0", alpha=0.95),
            zorder=10)

    # ---- 下半部分：详细解释区（独立 axes，带方框边框）----
    ax_exp = fig.add_axes([0.02, 0.003, 0.96, 0.605])
    ax_exp.axis("off")

    # 可见方框边框
    border = FancyBboxPatch((0.008, 0.008), 0.984, 0.984,
                             boxstyle="square,pad=0",
                             transform=ax_exp.transAxes,
                             facecolor="#f5f5f5", edgecolor="#888",
                             linewidth=1.5, zorder=1)
    ax_exp.add_patch(border)

    _fp = _font_simhei
    _c = "#222"
    _fs = 9.0
    _ls = 1.6       # 行间距: 紧凑但不重叠
    _pad = 0.018    # 段间距: 收紧

    def _t(y, txt, **kw):
        """全部居中"""
        default = dict(transform=ax_exp.transAxes, va="top", ha="center",
                       fontsize=_fs, fontproperties=_fp, color=_c,
                       linespacing=_ls, zorder=3, x=0.5)
        default.update(kw)
        ax_exp.text(default.pop("x", 0.5), y, txt, **default)

    y = 0.97

    # 标题
    _t(y, "图表逐元素详解 (从坐标轴到每条线到每个标注点的完整含义)",
       fontweight="bold", fontsize=10)
    y -= _pad * 1.4

    # [一] X轴
    _t(y, "[一] X轴含义: ratio = |dx| / |dy|   (线段的胖瘦比 / 陡峭度)")
    y -= _pad
    _t(y, "这是线段的「横向跨度 / 纵向跨度」。对数刻度(10^-1 到 10^1)把全部斜率均匀铺开。")
    y -= _pad
    _t(y, "ratio > 1 --> 线段横着(宽>高), 如 dx=60/dy=40 则 ratio=1.5, 规则要求驱动X")
    y -= _pad
    _t(y, "ratio < 1 --> 线段竖着(高>宽), 如 dx=40/dy=60 则 ratio=0.67, 规则要求驱动Y")
    y -= _pad
    _t(y, "ratio = 1 --> 正对角线(宽=高), 如 dx=50/dy=50, 两轴对称, 驱动谁效果一样")
    y -= _pad
    _t(y, "ratio很大(>10) --> 几乎水平;  ratio很小(<0.1) --> 几乎垂直")
    y -= _pad * 1.2

    # [二] Y轴
    _t(y, "[二] Y轴含义: 终点误差(px) = 实际停点到目标点的直线距离")
    y -= _pad
    _t(y, "终点误差 = sqrt((实际停点Px.x - 目标点E.x)^2 + (Px.y - E.y)^2)")
    y -= _pad
    _t(y, "含义: 你命令舵机走到E点, 但舵机是逐步跳着走的(每步7px), 跳过E附近时发现")
    y -= _pad
    _t(y, "     够近了(<4px死区)就停下来。这个够近但没正中的距离就是Y轴的值。")
    y -= _pad
    _t(y, "单位px(像素), 物理换算: 1px 约等于 2.84mm (PX_PER_M=352, 176px约等于0.5m)")
    y -= _pad * 1.2

    # [三] 三条线
    _t(y, "[三] 三条线的含义")
    y -= _pad
    _t(y, "[蓝实线] 固件按规则选轴 (当前代码的实际表现)")
    y -= _pad
    _t(y, "|dx|>=|dy| 时驱动X(x每步+7,y用calc_y公式算), 否则驱动Y(y每步+7,x用calc_x算)")
    y -= _pad
    _t(y, "蓝线全程压在低位(大部分0~5px) --> 按规则选轴时终点误差一直很小")
    y -= _pad
    _t(y, "数学原因: 驱动大轴时有效斜率<=1, 终点上界 = DEADZONE*sqrt(2) 约等于 5.66px")
    y -= _pad * 1.1
    _t(y, "[橙虚线] 固件反着选轴 (故意违反规则的假想实验)")
    y -= _pad
    _t(y, "本该驱动X的却去驱动Y, 反之亦然。用来量化选错轴的代价。")
    y -= _pad
    _t(y, "橙线在两端(ratio很大或很小时)爆炸式上升(可达30px+), 中间(ratio约=1)接近蓝线。")
    y -= _pad
    _t(y, "原因: 错选时死区落在小轴上, 从轴是大轴, |有效斜率|>>1, 误差被放大几十倍。")
    y -= _pad
    _t(y, "例: dx=10/dy=200(很竖), 错选驱动X --> Y误差 = |200/10|*4 = 80px!")
    y -= _pad * 1.1
    _t(y, "[绿点划线] 理想状态理论下限 (纯double浮点+仅死区约束, 不做int截断)")
    y -= _pad
    _t(y, "假设固件用完美精度运算, 仅保留进+-4px就停的死区机制。系统能达到的最好结果。")
    y -= _pad
    _t(y, "绿线几乎和蓝线重合, 偶尔略低 --> int截断额外误差仅约0~1px(很小)。")
    y -= _pad
    _t(y, "主要误差来源不是截断, 而是死区机制本身(驱动轴不需要精确到达目标)。")
    y -= _pad * 1.2

    # [四]
    _t(y, "[四] 图上的散点标注")
    y -= _pad
    _t(y, "蓝色圆点 = 中等差异样本按规则选轴的终点误差(落在蓝线上)")
    y -= _pad
    _t(y, "橙色三角 = 同一样本反着选轴的终点误差(落在橙线上)")
    y -= _pad
    _t(y, "每个标签旁的白底文字框标出样本参数(如dx=60/dy=40), 不会被线遮挡。")
    y -= _pad * 1.2

    # [五]
    _t(y, "[五] 右上角摘要框")
    y -= _pad
    _t(y, "列出三个关键样本: dx=60/dy=40(规则胜0.45px), dx=40/dy=60(平局), 对角线(对称平局)")
    y -= _pad
    _t(y, "平局说明: 整数截断+离散步长的离散巧合, 不代表选轴无所谓")
    y -= _pad * 1.2

    # [六]
    _t(y, "[六] 黑色竖虚线 (ratio=1)")
    y -= _pad
    _t(y, "标记正对角线位置。三条线在此处交叉/重合(两轴跨度相等时驱动谁效果相同)")
    y -= _pad * 1.2

    # [七] 结论
    _t(y, "[七] 工程结论")
    y -= _pad
    _t(y, "ratio<0.3或>3 (极端倾斜): 规则vs错选差距巨大(可达30px/85mm) --> 必须按规则")
    y -= _pad
    _t(y, "0.3~3 (中等差异, 如dx=60/dy=40): 差距微小(0~4px/0~11mm) --> 规则仍略优, 选错不致命")
    y -= _pad
    _t(y, "ratio=1 (正对角): 完全对称, 差距为零 --> 选谁都一样")
    y -= _pad
    _t(y, "==> 当前规则 |dx|>=|dy|-->驱动X 在全部区间都是最优或不劣的选择, 且自动兜底极端情况",
       fontweight="bold")

    fig.savefig(os.path.join(out_dir, "fig5_moderate_case.png"), dpi=200, bbox_inches="tight",
                facecolor="white", edgecolor="none")
    plt.close(fig)

    print("\nFigure saved: fig5_moderate_case.png")

if __name__ == "__main__":
    main()
