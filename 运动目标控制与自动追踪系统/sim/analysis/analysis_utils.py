# -*- coding: utf-8 -*-
"""
analysis_utils.py —— sim/analysis 公共绘图工具
================================================================================
解决两个历史痛点:
  1) 豆腐块: 中文字体统一探测 + 全局注册(fallback 链), 杜绝方块字
  2) 文字重叠/溢出: 顶部"本图目的"横幅 + 自动换行(CJK 宽度感知)的解释区

用法(各 gen_*.py 顶部替换为):
    import analysis_utils as au
    au.setup_chinese_font()          # 1) 全局字体配置(防豆腐块)
    ...
    au.add_banner(fig, "图N 标题", "一句话说明这张图的目的")   # 2a) 顶部横幅
    au.add_explanation(fig, [...], y0=0.02, h=0.38)           # 2b) 解释区(自动换行)
================================================================================
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyBboxPatch

# ---------------------------------------------------------------------------
# 1) 中文字体: 候选列表按优先级排列, 自动探测第一个存在的
#    [重要] 微软雅黑(msyh.ttc)优先于黑体(simhei.ttf):
#    SimHei 缺 U+2212(数学负号 −)字形, 会把 log 轴刻度 10⁻¹ 的负号渲染成方块;
#    雅黑含该字形, 且通过 fallback 链补齐其他缺失字形。
# ---------------------------------------------------------------------------
_FONT_CANDIDATES = [
    r"C:\Windows\Fonts\msyh.ttc",      # 微软雅黑 (优先: 字形全, 支持 U+2212)
    r"C:\Windows\Fonts\msyh.ttf",
    r"C:\Windows\Fonts\simhei.ttf",    # 黑体 (备用)
    r"C:\Windows\Fonts\simsun.ttc",    # 宋体
]
# rcParams 字体链: 首选字体 + 黑体 + DejaVu Sans(兜底数学符号)
_FALLBACK_CHAIN = ["Microsoft YaHei", "SimHei", "DejaVu Sans", "sans-serif"]

_FONT = None          # 共享 FontProperties 对象
_FONT_NAME = None     # 实际选中的字体名


def setup_chinese_font(verbose=True):
    """全局注册可用中文字体并写入 rcParams, 返回 FontProperties。
    找不到任何中文字体时返回 None(此时图内中文只能显示为方块, 会打印警告)。"""
    global _FONT, _FONT_NAME

    # 优先: 直接注册字体文件 (最可靠, 不依赖 matplotlib 缓存)
    for f in _FONT_CANDIDATES:
        if os.path.exists(f):
            try:
                font_manager.fontManager.addfont(f)
                _FONT = FontProperties(fname=f)
                _FONT_NAME = _FONT.get_name()
                break
            except Exception:
                continue

    # 兜底: 让 matplotlib 在已安装字体里按名称找
    if _FONT is None:
        for name in ["SimHei", "Microsoft YaHei", "SimSun", "sans-serif"]:
            try:
                font_manager.findfont(name, fallback_to_default=False)
                _FONT = FontProperties(family=name)
                _FONT_NAME = name
                break
            except Exception:
                continue

    if _FONT is not None:
        # 全局生效: 轴标签/标题/图例等不再需要逐个传 fontproperties
        # 注意: fallback 链里放 DejaVu Sans, 兜底 SimHei 缺的 U+2212 等数学符号
        plt.rcParams["font.sans-serif"] = [_FONT_NAME] + _FALLBACK_CHAIN
        plt.rcParams["axes.unicode_minus"] = False
        plt.rcParams["svg.fonttype"] = "path"      # 文字转路径, 防 SVG 迁移缺字
        if verbose:
            print("[字体] 已启用中文字体: %s" % _FONT_NAME)
    elif verbose:
        print("[警告] 未找到任何中文字体, 图片中文将显示为方块!")
    return _FONT


def F():
    """返回共享字体 FontProperties(显式传 fontproperties 时用)。"""
    return _FONT


# ---------------------------------------------------------------------------
# CJK 感知换行: 中文/全角字符占 2 个西文字符宽度
# ---------------------------------------------------------------------------
def wrap_cjk(text, width=70):
    """按显示宽度换行: CJK/全角=2, 西文/半角=1。返回换行后的行列表。"""
    lines, cur, cur_w = [], "", 0
    for ch in text:
        w = 2 if ord(ch) > 0x2E80 else 1
        if cur_w + w > width and cur:
            lines.append(cur)
            cur, cur_w = ch, w
        else:
            cur += ch
            cur_w += w
    if cur:
        lines.append(cur)
    return lines


# ---------------------------------------------------------------------------
# 2a) 顶部"本图目的"横幅
# ---------------------------------------------------------------------------
def add_banner(fig, title, purpose, fs_title=11.5, fs_body=9.0):
    """在 figure 顶部画横幅: [本图目的] 标题 + 一句话说明(超长自动换行)。
    占位 y ∈ [0.93, 1.0], 主图请勿越过 y=0.90。"""
    ax = fig.add_axes([0.01, 0.93, 0.98, 0.07])
    ax.axis("off")
    border = FancyBboxPatch((0.004, 0.04), 0.992, 0.92,
                            boxstyle="round,pad=0.02",
                            transform=ax.transAxes,
                            facecolor="#e8f0fe", edgecolor="#3f6fb5",
                            linewidth=1.4, zorder=1)
    ax.add_patch(border)

    body_fp = _FONT or FontProperties()
    # 标题(粗体)
    ax.text(0.015, 0.80, "本图目的:", transform=ax.transAxes, va="center",
            fontsize=fs_title, fontproperties=body_fp, color="#1a3a6b",
            fontweight="bold", zorder=3)
    ax.text(0.135, 0.80, title, transform=ax.transAxes, va="center",
            fontsize=fs_title, fontproperties=body_fp, color="#1a3a6b",
            fontweight="bold", zorder=3)
    # 正文(自动换行, 最多 2 行)
    wrapped = wrap_cjk(purpose, width=118)
    shown = wrapped[:2]
    if len(wrapped) > 2:
        shown[1] = shown[1] + "…"
    for i, line in enumerate(shown):
        ax.text(0.015, 0.44 - i * 0.34, line, transform=ax.transAxes,
                va="center", fontsize=fs_body,
                fontproperties=body_fp, color="#222", zorder=3)
    return ax


# ---------------------------------------------------------------------------
# 2b) 解释区: 带边框 + 自动换行 + 自动行高, 杜绝重叠/溢出
# ---------------------------------------------------------------------------
def add_explanation(fig, lines, y0=0.02, h=0.40, fs=8.5,
                    lh_ratio=1.55, wrap_width=92, title=None,
                    bg="#fafbfc", edge="#999"):
    """在 figure 底部画解释区。

    参数
    ----
    lines      : list[str] 或 list[tuple[str, bool]], (文本, 是否粗体)
    y0, h      : 解释区左下角 y 与高度 (axes 比例)
    fs         : 字号
    lh_ratio   : 行高 = 字号 × 该系数 (自动防重叠的关键)
    wrap_width : 单行最大显示宽度 (CJK 宽度单位)
    title      : 解释区顶部小标题 (可选, 如 "详细说明")
    """
    fp = _FONT or FontProperties()

    # 先展开成 (text, bold) 列表, 并做 CJK 换行
    flat = []
    for item in lines:
        if isinstance(item, tuple):
            txt, bold = item
        else:
            txt, bold = item, False
        for seg in wrap_cjk(txt, wrap_width):
            flat.append((seg, bold))

    n = len(flat) + (1 if title else 0)
    if n == 0:
        return None

    # 行高: 基于字号与 axes 高度反算, 保证不重叠
    fig_h = fig.get_size_inches()[1] * h
    line_h = fs / 72.0 * lh_ratio / fig_h      # 每行占 axes 高度比例
    pad = 0.012

    if (n * line_h + pad * 2) > 1.0:
        # 文字太多放不下: 缩小字号直到能放下 (下限 6pt)
        fs = max(6.0, 0.9 * fs)
        line_h = fs / 72.0 * lh_ratio / fig_h

    ax = fig.add_axes([0.0, y0, 1.0, h])
    ax.axis("off")
    border = FancyBboxPatch((0.005, 0.02), 0.99, 0.96,
                            boxstyle="square,pad=0",
                            transform=ax.transAxes,
                            facecolor=bg, edgecolor=edge,
                            linewidth=1.2, zorder=1)
    ax.add_patch(border)

    y = 1.0 - pad
    if title:
        ax.text(0.03, y, title, transform=ax.transAxes, va="top", ha="left",
                fontsize=fs + 1.5, fontproperties=fp, color="#333",
                fontweight="bold", zorder=3)
        y -= line_h * 1.35

    for txt, bold in flat:
        ax.text(0.03, y, txt, transform=ax.transAxes, va="top", ha="left",
                fontsize=fs, fontproperties=fp, color="#222",
                fontweight="bold" if bold else "normal", zorder=3)
        y -= line_h
    return ax


# ---------------------------------------------------------------------------
# 3) 图下方统一"页码/生成脚本"水印 (可选)
# ---------------------------------------------------------------------------
def add_footer(fig, script_name, extra=""):
    fp = _FONT or FontProperties()
    fig.text(0.01, 0.005, "脚本: %s%s" % (script_name, extra),
             fontsize=7, fontproperties=fp, color="#888", ha="left", va="bottom")
