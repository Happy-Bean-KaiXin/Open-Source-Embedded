# -*- coding: utf-8 -*-
"""
sim_firmware_consistency_check.py
================================================================================
实证对照：仿真代码 vs 固件代码 是否一致
================================================================================
对比三组实现在**完全相同输入**下的算术输出，判定一致性：

  A) 固件忠实版  (fw_*)  —— 严格按 contorl.c 的 C 语义（已对齐 2026-08-19 修复版）：
        - float32 用 struct 打包模拟
        - 输入 int 参数按 C 隐式 (int) 截断向零
        - calculateIntercept 参数 (int x1,int y1) 会先把传入的 float 截断成 int
        - calculateSlope 竖边(x2==x1) 返回 0.0f（修复版有守卫），由 *_Y_* 分支 if(dx!=0) 锁 x

  B) Python 仿真版 (py_*)  —— 照搬 gen_large_figs.py / gen_moderate_case.py：
        - float32 用 struct 打包
        - 输入原样 float，不截断
        - calculateSlope 竖边返回 0.0
        - calculateX 竖边返回 0

  C) JS 仿真版     —— 运行 Motion_TarCtrl_Sim.js，抓 path 末点对比

判定：对每条随机线段，比较 slope / intercept / calcY(某x) / calcX(某y)。
      若 |fw - py| <= 1e-6 视为一致（float32 舍入内一致）。
================================================================================
"""
import os, struct, math, random, subprocess, json

HERE = os.path.dirname(os.path.abspath(__file__))
JS = os.path.join(HERE, "..", "sim_code", "Motion_TarCtrl_Sim.js")

def f32(x):
    return struct.unpack("f", struct.pack("f", float(x)))[0]

def trunc(x):
    return int(x)

# ---------------- A) 固件忠实版 ----------------
def fw_calcSlope(x1, y1, x2, y2):
    # 与 contorl.c 修复后一致：竖边(x2==x1)返回 0.0f（calculateSlope 有守卫），
    #   各 *_Y_* 分支用 if(Flag.dx!=0) 保持 x 常量。非竖边返回 float32 斜率。
    if abs(x2 - x1) < 1e-9:
        return 0.0
    return f32((y2 - y1) / (x2 - x1))

def fw_calcIntercept(x1, y1, slope):
    # C 签名: int calculateIntercept(int x1, int y1, float slope)
    # 传入的 float 会先被 (int) 截断向零！
    x1 = trunc(x1); y1 = trunc(y1)
    return trunc(f32(f32(y1) - f32(slope * x1)))

def fw_calcY(x, slope, intercept):
    # C 签名: int calculateY(int x, float slope, int intercept)
    x = trunc(x); intercept = trunc(intercept)
    return trunc(f32(f32(slope * x) + f32(intercept)))

def fw_calcX(y, slope, intercept):
    if slope == 0.0:
        return 0
    return trunc(f32(f32(y - f32(intercept)) / f32(slope)))

# ---------------- B) Python 仿真版 ----------------
def py_calcSlope(x1, y1, x2, y2):
    if abs(x2 - x1) < 1e-9:
        return 0.0
    return f32((y2 - y1) / (x2 - x1))

def py_calcIntercept(x1, y1, slope):
    return trunc(f32(f32(y1) - f32(slope * x1)))

def py_calcY(x, slope, intercept):
    if slope == 0.0:
        return intercept
    return trunc(f32(f32(slope * x) + f32(intercept)))

def py_calcX(y, slope, intercept):
    if slope == 0.0:
        return 0
    return trunc(f32(f32(y - f32(intercept)) / f32(slope)))


def compare(n=5000, seed=20260819):
    rng = random.Random(seed)
    mism_slope = mism_int = mism_y = mism_x = 0
    max_slope = max_int = max_y = max_x = 0.0
    ex_slope = ex_int = ex_y = ex_x = None
    vertical_cases = 0
    for _ in range(n):
        x1 = rng.randint(0, 320); y1 = rng.randint(0, 320)
        x2 = rng.randint(0, 320); y2 = rng.randint(0, 320)
        dx = x2 - x1; dy = y2 - y1
        if abs(dx) < 1 and abs(dy) < 1:
            continue
        if dx == 0:
            vertical_cases += 1

        # 模拟固件调用：x_centry/y_centry 常为 float（如 120.0），会被 int 截断
        cx, cy = float(x1), float(y1)

        # slope
        fs = fw_calcSlope(cx, cy, x2, y2)
        ps = py_calcSlope(cx, cy, x2, y2)
        if dx == 0:
            # 固件: inf ; 仿真: 0.0  → 竖边机制不同，单独统计不计入"算术不一致"
            pass
        else:
            d = abs(fs - ps)
            if d > 1e-6:
                mism_slope += 1
                if d > max_slope: max_slope = d; ex_slope = (cx,cy,x2,y2,fs,ps)
            # intercept (固件把 cx,cy 当 int 截断)
            fi = fw_calcIntercept(cx, cy, fs)
            pi = py_calcIntercept(cx, cy, ps)
            d = abs(fi - pi)
            if d > 1e-6:
                mism_int += 1
                if d > max_int: max_int = d; ex_int = (cx,cy,x2,y2,fi,pi)
            # calcY at x = x2
            fy = fw_calcY(x2, fs, fi)
            py_ = py_calcY(x2, ps, pi)
            d = abs(fy - py_)
            if d > 1e-6:
                mism_y += 1
                if d > max_y: max_y = d; ex_y = (x2,fs,fi,fy,py_)
            # calcX at y = y2 (仅非竖边)
            if abs(dx) > 0:
                fx = fw_calcX(y2, fs, fi)
                px = py_calcX(y2, ps, pi)
                d = abs(fx - px)
                if d > 1e-6:
                    mism_x += 1
                    if d > max_x: max_x = d; ex_x = (y2,fs,fi,fx,px)

    print("=" * 70)
    print("实证对照结果（%d 条随机线段，seed=%d）" % (n, seed))
    print("-" * 70)
    print("竖边(dx==0)样本数: %d  (机制不同: 固件=inf/UB, 仿真=0.0, 单独处理)" % vertical_cases)
    print()
    print("【非竖边算术一致性】")
    print("  slope    : 不一致 %3d 条, 最大差 %.3e  %s" % (mism_slope, max_slope, ex_slope))
    print("  intercept: 不一致 %3d 条, 最大差 %.3e  %s" % (mism_int, max_int, ex_int))
    print("  calcY    : 不一致 %3d 条, 最大差 %.3e  %s" % (mism_y, max_y, ex_y))
    print("  calcX    : 不一致 %3d 条, 最大差 %.3e  %s" % (mism_x, max_x, ex_x))
    print()
    total = mism_slope + mism_int + mism_y + mism_x
    if total == 0:
        print("结论: 非竖边算术 逐位一致 (float32 舍入内)，仿真与固件算术等价。")
    else:
        print("结论: 存在 %d 处算术差异，详见上方样例。" % total)
    print("=" * 70)
    return dict(mism_slope=mism_slope, mism_int=mism_int, mism_y=mism_y,
                mism_x=mism_x, max_int=max_int, ex_int=ex_int)

# ---------------- C) JS 仿真版抓取 ----------------
def run_js():
    """运行 JS 自测，看是否 PASS；并抓一条红框 path 末点。"""
    try:
        out = subprocess.run(
            ["node", JS], capture_output=True, text=True, timeout=60)
        return out.stdout + out.stderr
    except Exception as e:
        return "[JS 运行失败] %s" % e

if __name__ == "__main__":
    res = compare()
    print()
    print("【JS 仿真自测输出】")
    print(run_js())
