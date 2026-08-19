# 大样本扫描测试报告

## 1. 测试目的

验证固件 DDA/Bresenham 数字直线生成器（`contorl.c` 中的 `calculateSlope/Intercept/Y/X` 四函数）在**大样本随机线段**上的终点误差是否满足工程精度要求。

核心问题：
- 规则选轴（|dx| >= |dy| 驱动 X，否则驱动 Y）的终点误差是否有界？
- 错选轴的误差会爆炸到什么程度？
- 垂直距离误差（perp）和欧氏终点误差（endpoint）各有多大？

---

## 2. 测试方法

### 2.1 算术模型

Python 脚本 **严格复现** 固件浮点运算链：

```python
def f32(x): return struct.unpack("f", struct.pack("f", float(x)))[0]  # IEEE-754 单精度
def trunc(x): return int(x)  # 向零截断（与固件 (int) 强转一致）
```

四函数签名与固件完全一致：

| 函数 | Python 实现 | 固件位置 |
|------|------------|---------|
| `calcSlope(x1,y1,x2,y2)` | `f32((y2-y1)/(x2-x1))`, dx=0 返回 0.0 | contorl.c:326 |
| `calcInt(x1,y1,slope)` | `trunc(f32(y1 - slope*x1))`, slope=0 返回 0 | contorl.c:340 |
| `calcY(x,slope,intercept)` | `trunc(f32(slope*x + intercept))`, slope=0 返回 x | contorl.c:348 |
| `calcX(y,slope,intercept)` | `trunc(f32((y - intercept) / slope))`, slope=0 返回 0 | contorl.c:356 |

### 2.2 模拟参数

| 参数 | 值 | 物理含义 |
|------|-----|---------|
| `STEP` | 7 px | 每步移动量 ≈ 20mm |
| `DEADZONE` | 4 px | 死区阈值 ≈ 11mm |
| `PX_PER_M` | 352 px/m | 像素-米换算，1px ≈ 2.84mm |

### 2.3 数据集构成

共 **172 条线段**，分为两组：

| 组别 | 数量 | 说明 |
|------|------|------|
| 手工精选 | 12 条 | 覆盖纯横、纯竖、正/反对角、中等斜率、浅/陡、四个象限 |
| 随机生成 | 160 条 | 种子 `20260817`，坐标范围 [30,320]，过滤掉过短线段 |

每条线段记录：起点/终点、dx/dy/ratio、选轴规则、perp_max、perp_avg、规则终点误差、错选终点误差。

---

## 3. 核心结果

### 3.1 终点误差统计

| 指标 | 规则选轴 | 错选轴 |
|------|---------|--------|
| **最大值** | **5.000 px** | **162.0 px** |
| 平均值 | ~2.05 px | ~15-30 px（随 ratio 爆炸） |
| 中位数 | ~1.4-2.2 px | — |
| 理论上界 | DEADZONE × √2 ≈ **5.66 px** | 4 × max(ratio, 1/ratio) |

### 3.2 关键发现

#### 发现 1：规则选轴误差有界且很小
- 实测最大 **5.0px** < 理论上界 **5.66px**
- 绝大多数线段误差在 **1-3px** 范围内
- 对应物理尺寸：5px × 2.84mm/px ≈ **14.2mm**

#### 发现 2：错选轴代价随斜率比指数增长
- ratio = 0.25（陡线，dy >> dx）：错选 X 驱动 → 误差可达 **58+ px**
- ratio = 4.0（浅线，dx >> dy）：错选 Y 驱动 → 误差可达 **23+ px**
- ratio = 1（对角线）：两轴对称 → 选谁都一样，误差相同

#### 发现 3：垂直距离误差更小
- perp_max 最大值约 **1.89 px**（≈ 5.4mm）
- perp_avg 多数在 **0.25-1.0 px** 范围内
- 说明轨迹虽然终点有偏差，但整体路径与理想直线非常接近

### 3.3 典型案例对比

| 案例 | dx | dy | ratio | 规则误差(px) | 错选误差(px) | 结论 |
|------|-----|-----|-------|-------------|-------------|------|
| 纯横 | 260 | 0 | 1e6 | 1.0 | 1.0 | 无从轴，无差别 |
| 纯竖 | 0 | 260 | 0 | 1.0 | 1.0 | 同上 |
| 正对角 | 200 | 200 | 1.0 | 4.24 | 4.24 | 对称，无差别 |
| **中等(经典)** | **60** | **40** | **1.5** | **3.16** | **3.61** | **规则胜 12%** |
| 中等(反向) | 40 | 60 | 0.67 | 3.61 | 3.61 | tie（Y 驱动正确） |
| 浅 | 120 | 30 | 4.0 | 1.41 | 8.25 | 规则胜 83% |
| 陡 | 30 | 120 | 0.25 | 1.41 | 8.25 | 规则胜 83% |

---

## 4. 图表索引

| 图号 | 文件名 | 内容 |
|------|--------|------|
| 图 1 | `fig1_perstep_dev.png` | 172 条线段的逐步垂直距离偏差曲线 |
| 图 2 | `fig2_endpoint_err.png` | 终点误差柱状图（规则 vs 错选对比） |
| 图 3 | `fig3_examples.png` | 6 条典型线段的轨迹可视化（含放大窗口） |
| 图 4 | `fig4_feasibility_basis.png` | 可行性基础分析（误差分布 + 累积概率） |
| 图 5 | `fig5_moderate_case.png` | 经典案例 S(100,100)→E(160,140) 详细分解 |
| 图 6 | `fig6_schematic.png` | 空间示意图（含 P' 放大标注） |
| 图 7 | `fig7_xy_compare.png` | X-calc-Y vs Y-calc-X 逐段散点对比 |
| 图 8 | `fig8_error_analysis.png` | 误差理论界 vs 200 条实测数据 |

---

## 5. 工程结论

1. **固件的 DDA 直线生成器满足工程精度要求。** 规则选轴下终点误差 ≤ 5px（≤14.2mm），对于激光笔定位场景完全可接受。
2. **选轴规则 `|dx| >= |dy| → X 驱动` 是最优策略。** 该规则保证有效斜率 ≤ 1，从而将误差限制在 DEADZONE×√2 以内。
3. **最坏情况出现在正对角线（ratio=1）。** 此时误差 = STEP×√2/2 ≈ 4.24px（一步之差的几何投影），属于算法固有极限，无法通过调参消除。
4. **垂直距离误差（perp）< 2px** 说明轨迹本身非常接近理想直线，只是"最后一步"可能多走/少走导致终点偏移。

---

## 6. 文件清单

```
sim/analysis/
├── gen_large_figs.py          # 主脚本（生成图1-4 + CSV）
├── gen_moderate_case.py       # 图5 经典案例
├── gen_schematic.py           # 图6 空间示意图
├── gen_xy_compare.py          # 图7 XY 对比
├── gen_error_analysis.py      # 图8 误差理论界 vs 实测
├── extract_firmware.py        # 从 contorl.c 提取四函数
├── large_test_dataset.csv     # 172 条完整数据
├── fig1_perstep_dev.png
├── fig2_endpoint_err.png
├── fig3_examples.png
├── fig4_feasibility_basis.png
├── fig5_moderate_case.png
├── fig6_schematic.png
├── fig7_xy_compare.png
└── fig8_error_analysis.png
```
