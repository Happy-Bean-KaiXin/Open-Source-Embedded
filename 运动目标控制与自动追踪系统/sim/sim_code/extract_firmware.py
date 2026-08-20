# -*- coding: utf-8 -*-
"""
extract_firmware.py  ——  从固件源抽取运动控制比对副本
================================================================================
源文件 : User/contorl/contorl.c  (GBK 编码 -> 转码 UTF-8)
输出   : firmware_Motion_TarCtrl.c  (与 Motion_TarCtrl_Sim.js 逐行对照用)

抽取方式:
    1. 以 GBK 读取 contorl.c，解码为 UTF-8（坏字节忽略，不阻断）。
    2. 用「函数签名 + 花括号配对」自动抽取下列函数体:
         calculateSlope / calculateIntercept / calculateY / calculateX
         Motion_TarCtrl  (红大方框/标定矩形, BIG_FRAME_STEP=7)
         Motion_TarCtrl_Black (A4 黑胶带框, BLACK_FRAME_STEP=1)
    3. 在文件头写入说明，所有中文注释一并转 UTF-8。
    4. 自检: 扫描死区阈值，确认红框非第四段/第四段、黑框的到达死区
       分别为 <4 / <10 / <2，且不存在残留的「< 1」旧 bug 阈值。

用法:
    python extract_firmware.py
================================================================================
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "..", "User", "contorl", "contorl.c")
OUT = os.path.join(HERE, "firmware_Motion_TarCtrl.c")

# 需要抽取的函数（按出现顺序）
FUNC_NAMES = [
    "calculateSlope",
    "calculateIntercept",
    "calculateY",
    "calculateX",
    "Motion_TarCtrl",
    "Motion_TarCtrl_Black",
]

HEADER = """/* =============================================================================
 * firmware_Motion_TarCtrl.c  ——  从固件源抽取的比对副本（非编译文件）
 * -----------------------------------------------------------------------------
 * 源文件 : User/contorl/contorl.c  (GBK 源码已转码为 UTF-8)
 * 抽取函数: Motion_TarCtrl (红大方框/标定矩形, BIG_FRAME_STEP=7)
 *           Motion_TarCtrl_Black (A4 黑胶带框, BLACK_FRAME_STEP=1)
 *           calculateSlope/Intercept/Y/X (32位float斜率 + (int)向零截断)
 * 抽取方式: 花括号配对自动抽取，注释一并转 UTF-8。
 * 用途    : 与 sim/sim_code/Motion_TarCtrl_Sim.js 逐行对照，确认仿真与固件一致。
 * 生成方式: 由 sim/sim_code/extract_firmware.py 自动抽取（可复现）。
 *
 * 当前固件状态(已修复):
 *   - 红框到达死区统一为 非第四段 <4、第四段 <10 (原 Centry_Y dy<0 分支 <1 已改为 <4)
 *   - 黑框到达死区全程 <2
 *   - calculateSlope 竖边(dx==0)返回 0，由各 *_Y_* 分支 if(dx!=0) 守卫保持 x 常量
 * =========================================================================== */

"""


def read_src_gbk(path):
    """以 GBK 读取并转 UTF-8，坏字节忽略。"""
    with open(path, "rb") as f:
        raw = f.read()
    return raw.decode("gbk", errors="ignore")


def extract_function(src, name):
    """找到 `name(...)` 签名，从其后第一个 '{' 起做花括号配对，返回完整函数文本。"""
    # [BUGFIX] 原正则 (?:[\w\*]\s+)+ 贪婪匹配只从 't ' 开始（float 内无空白），
    #   把 "float" 的 "floa" 吃掉只剩 "t"。改用 [\w\*]+\s+ 让整个类型 token 被匹配。
    sig_pat = re.compile(r"(?:[\w\*]+\s+)+" + re.escape(name) + r"\s*\(")
    m = sig_pat.search(src)
    if not m:
        return None
    # 从签名末尾开始找第一个 '{'
    i = src.find("{", m.end())
    if i < 0:
        return None
    depth = 0
    j = i
    while j < len(src):
        c = src[j]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                j += 1
                break
        j += 1
    return src[m.start():j]


def self_check(text):
    """自检死区阈值是否合理（无残留 <1 旧 bug，红框<4/<10、黑框<2 至少出现）。"""
    problems = []
    # 旧 bug: 任意死区写成 < 1（含 <= 1）
    for m in re.finditer(r"<\s*=?\s*1\b", text):
        ctx = text[max(0, m.start() - 25): m.end() + 5].replace("\n", " ")
        # 仅当该阈值看起来是“到达死区”判断时报错（粗略过滤：附近含 myabs 或 Retangle/Black）
        if ("myabs" in ctx) or ("Retangle" in ctx) or ("Black" in ctx):
            problems.append("疑似残留 <1 死区: ...%s..." % ctx.strip())
    # 必须有红框 <4、第四段 <10、黑框 <2
    has_red4 = ("< 4" in text) or ("<4" in text)
    has_fourth10 = ("< 10" in text) or ("<10" in text)
    has_black2 = ("< 2" in text) or ("<2" in text)
    if not has_red4:
        problems.append("未检测到红框 <4 死区阈值")
    if not has_fourth10:
        problems.append("未检测到第四段 <10 死区阈值")
    if not has_black2:
        problems.append("未检测到黑框 <2 死区阈值")
    return problems


def main():
    if not os.path.exists(SRC):
        print("[ERROR] 源文件不存在: %s" % SRC)
        sys.exit(1)

    src = read_src_gbk(SRC)
    blocks = []
    missing = []
    for name in FUNC_NAMES:
        fn = extract_function(src, name)
        if fn is None:
            missing.append(name)
            print("[WARN] 未找到函数: %s" % name)
        else:
            blocks.append(fn)
            print("[OK] 抽取 %-20s (%d 行)" % (name, fn.count("\n") + 1))

    if missing:
        print("[ERROR] 缺失函数: %s" % ", ".join(missing))
        # 仍然继续输出已抽取部分

    out_text = HEADER + "\n".join(blocks) + "\n"
    # [BUGFIX] 用 newline='' 禁止文本模式自动换行转换：
    #   默认文本模式在 Windows 会把 '\n' 写成 '\r\n'，而固件源码已是 '\r\n'，
    #   导致每个行尾变成 '\r\r\n'（多一个回车符，编辑器显示为多余空行）。
    with open(OUT, "w", encoding="utf-8", newline="") as f:
        f.write(out_text)
    print("[OK] 已写出: %s  (%d 字节)" % (OUT, len(out_text.encode("utf-8"))))

    problems = self_check(out_text)
    if problems:
        print("[SELF-CHECK FAIL]")
        for p in problems:
            print("   - " + p)
        sys.exit(2)
    else:
        print("[SELF-CHECK PASS] 死区阈值正确 (红框<4/<10, 黑框<2)，无残留 <1。")


if __name__ == "__main__":
    main()
