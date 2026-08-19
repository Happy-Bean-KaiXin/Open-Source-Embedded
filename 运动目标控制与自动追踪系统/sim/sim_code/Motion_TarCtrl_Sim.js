/* =============================================================================
 * Motion_TarCtrl_Sim / Motion_TarCtrl_Black_Sim  —— 仿真版运动控制器
 * -----------------------------------------------------------------------------
 * 与固件 User/contorl/contorl.c 的两个函数【逐行对应】：
 *   Motion_TarCtrl_Sim       ↔  固件 Motion_TarCtrl       （红大方框 / 标定矩形，BIG_FRAME_STEP=7）
 *   Motion_TarCtrl_Black_Sim ↔  固件 Motion_TarCtrl_Black （A4 黑胶带框，BLACK_FRAME_STEP=1）
 *
 * 本文件自包含（含几何辅助 + 状态枚举），可直接 node 运行验证逻辑：
 *   node Motion_TarCtrl_Sim.js
 *
 * 仿真侧与固件侧“逐行对照”说明：
 *   - 完全一致：状态流转 BOX→ANY→CROSS→各 X/Y 子状态→回角点0；逐边顺序 0→1→2→3→0；
 *     calcSlope/calcInt/calcY/calcX 公式；竖向边处理（仿真 fixed_x ↔ 固件 dx!=0 守卫）。
 *   - 阈值：大方框 非第四段<4、第四段<10；黑框 全程<2（均取自固件，见 contorl.c 行 378/446/571/632 等）。
 *   - 黑框起点：#if BLACK_FRAME_START_FROM_CORNER 宏 → JS 用 startFromCorner 参数；
 *     非角点起步时起点取黑框自身四角几何中心 bx_c/by_c（BUG2 修复），与固件一致。
 *   - 唯一“实现不同但结果等价”处：固件 calculateSlope 在竖边返回 0、各 *_Y_* 用 if(dx!=0) 守卫；
 *     仿真 calcSlope 返回 Infinity、用 fixed_x 锁常量。两者竖边均“x 恒为常量、y 变化”。
 *
 * 【2026-08-17 同步更新】已对齐修复后的固件：
 *   - 到达死区阈值统一为 红框<4/<10、黑框<2（原 Centry_Y dy<0 分支 <1 已修正为 <4）。
 *   - 几何辅助用 Math.fround 模拟固件 32 位 float、用 Math.trunc 模拟 (int) 截断，
 *     与固件 calculateSlope/Intercept/Y/X 算术逐行一致，可直接与 firmware_Motion_TarCtrl.c 比对。
 * ============================================================================= */

/* ---------------- 几何/数学辅助（与固件 calculateSlope/Intercept/Y/X 逐行对应） ----------------
 * 关键：固件用 32 位 float 做斜率运算、用 (int) 截断（向零取整）返回整数坐标。
 *       仿真侧用 Math.fround 模拟 float32、用 Math.trunc 模拟 (int) 截断，保证“逐行对应”可比较。
 * 竖边(dx==0)：固件 calculateSlope 返回 0.0f，由各 *_Y_* 分支 if(dx!=0) 守卫保持 x 常量；
 *             仿真侧保持 Infinity + fixed_x 锁（结果等价：竖边 x 恒为常量、y 步进）。 */
function myabs(p){ return p > 0 ? p : -p; }
function f32(v){ return Math.fround(v); }   // 模拟固件 32 位 float
function calcSlope(x1,y1,x2,y2){ return (Math.abs(x2-x1) < 1e-9) ? Infinity : f32((y2-y1)/(x2-x1)); }
function calcInt(x1,y1,slope){ if(!isFinite(slope)) return 0; return Math.trunc(f32(f32(y1) - f32(slope * x1))); }
function calcY(x,slope,intercept){ if(!isFinite(slope)) return intercept; return Math.trunc(f32(f32(slope * x) + intercept)); }
function calcX(y,slope,intercept){ if(!isFinite(slope)) return 0; return Math.trunc(f32(f32(y - intercept) / slope)); }

/* ---------------- 状态枚举（与 contorl.h / 固件一致） ---------------- */
const BOX=0, ANY=1, CROSS=2;
const CENTRY_X=3, CENTRY_Y=4, START_X=5, START_Y=6, SECOND_X=7, SECOND_Y=8,
      THRID_X=9, THRID_Y=10, FOURTH_X=11, FOURTH_Y=12;
const F_CENTRY=0, F_START=1, F_SECOND=2, F_THRID=3, F_FOURTH=4;

/* =============================================================================
 * 仿真版 Motion_TarCtrl  —— 对应固件 Motion_TarCtrl（红大方框）
 * 与固件一致：Box_Square_State → Any_Rectang_Box_State(Centy_To_Start 用传入中心 Cx,Cy)
 *            → Cross_State(分发) → Centry_X/Y_Start → Start_X/Y_Second → ... → Fourth_X/Y_End(回角点0)。
 * 完成阈值：非第四段 <4、第四段 <10（与 contorl.c 中 BIG_FRAME_STEP 路径一致）。
 * ============================================================================= */
function Motion_TarCtrl_Sim(RX, RY, Cx, Cy, step, bugMode, maxTicks){
  const F = {
    x_centry:Cx, y_centry:Cy,
    FSTATE: F_CENTRY,
    x_actual:Cx, y_actual:Cy,
    Slope:0, Intercpet:0,
    fixed_x:null, dx:0, dy:0, Is_10ms_YES:1,
    X_AXIS:0, Y_AXIS:0
  };
  let Laser_State = BOX;
  const path = [];
  let doneTick = -1;
  const TH_NON = 4, TH_FOURTH = 10;   // 非第四段<4、第四段<10（与 contorl.c 一致）

  function line_x(y){
    if(!bugMode && F.fixed_x !== null) return F.fixed_x;   // 修正版：竖边保持 x 常量
    return calcX(y, F.Slope, F.Intercpet);                 // 旧版：竖边 (y-int)/inf ≈ 0
  }

  for(let tick=0; tick<maxTicks; tick++){
    F.Is_10ms_YES = 1;

    if(Laser_State === BOX){
      Laser_State = ANY;
    } else if(Laser_State === ANY){
      if(F.FSTATE === F_CENTRY){
        F.Slope = calcSlope(F.x_centry,F.y_centry,RX[0],RY[0]);
        F.fixed_x = (F.Slope===Infinity)? F.x_centry : null;
        F.Intercpet = calcInt(F.x_centry,F.y_centry,F.Slope);
        F.dx = RX[0]-F.x_centry; F.dy = RY[0]-F.y_centry;
        Laser_State = CROSS;
      } else if(F.FSTATE === F_START){
        F.Slope = calcSlope(RX[0],RY[0],RX[1],RY[1]);
        F.fixed_x = (F.Slope===Infinity)? RX[0] : null;
        F.Intercpet = calcInt(RX[0],RY[0],F.Slope);
        F.dx = RX[1]-RX[0]; F.dy = RY[1]-RY[0];
        Laser_State = CROSS;
      } else if(F.FSTATE === F_SECOND){
        F.Slope = calcSlope(RX[1],RY[1],RX[2],RY[2]);
        F.fixed_x = (F.Slope===Infinity)? RX[1] : null;
        F.Intercpet = calcInt(RX[1],RY[1],F.Slope);
        F.dx = RX[2]-RX[1]; F.dy = RY[2]-RY[1];
        Laser_State = CROSS;
      } else if(F.FSTATE === F_THRID){
        F.Slope = calcSlope(RX[2],RY[2],RX[3],RY[3]);
        F.fixed_x = (F.Slope===Infinity)? RX[2] : null;
        F.Intercpet = calcInt(RX[2],RY[2],F.Slope);
        F.dx = RX[3]-RX[2]; F.dy = RY[3]-RY[2];
        Laser_State = CROSS;
      } else if(F.FSTATE === F_FOURTH){
        F.Slope = calcSlope(RX[3],RY[3],RX[0],RY[0]);
        F.fixed_x = (F.Slope===Infinity)? RX[3] : null;
        F.Intercpet = calcInt(RX[3],RY[3],F.Slope);
        F.dx = RX[0]-RX[3]; F.dy = RY[0]-RY[3];
        Laser_State = CROSS;
      }
    } else if(Laser_State === CROSS){
      if(F.FSTATE === F_CENTRY){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=F.x_centry; F.y_actual=F.y_centry; Laser_State=CENTRY_X; }
        else { F.x_actual=F.x_centry; F.y_actual=F.y_centry; Laser_State=CENTRY_Y; }
      } else if(F.FSTATE === F_START){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=RX[0]; F.y_actual=RY[0]; Laser_State=START_X; }
        else { F.y_actual=RY[0]; F.x_actual=RX[0]; Laser_State=START_Y; }
      } else if(F.FSTATE === F_SECOND){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=RX[1]; F.y_actual=RY[1]; Laser_State=SECOND_X; }
        else { F.x_actual=RX[1]; F.y_actual=RY[1]; Laser_State=SECOND_Y; }
      } else if(F.FSTATE === F_THRID){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=RX[2]; F.y_actual=RY[2]; Laser_State=THRID_X; }
        else { F.x_actual=RX[2]; F.y_actual=RY[2]; Laser_State=THRID_Y; }
      } else if(F.FSTATE === F_FOURTH){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=RX[3]; F.y_actual=RY[3]; Laser_State=FOURTH_X; }
        else { F.x_actual=RX[3]; F.y_actual=RY[3]; Laser_State=FOURTH_Y; }
      }
    } else if(Laser_State === CENTRY_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[0])<TH_NON){ F.y_actual=RY[0]; F.x_actual=RX[0]; F.FSTATE=F_START; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[0])<TH_NON){ F.y_actual=RY[0]; F.x_actual=RX[0]; F.FSTATE=F_START; Laser_State=ANY; } } }
    } else if(Laser_State === CENTRY_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step;
        if(myabs(F.y_actual-RY[0])<TH_NON){ F.x_actual=RX[0]; F.y_actual=RY[0]; F.FSTATE=F_START; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step;
        if(myabs(F.y_actual-RY[0])<TH_NON){ F.x_actual=RX[0]; F.y_actual=RY[0]; F.FSTATE=F_START; Laser_State=ANY; } } }
    } else if(Laser_State === START_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[1])<TH_NON){ F.y_actual=RY[1]; F.x_actual=RX[1]; F.FSTATE=F_SECOND; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[1])<TH_NON){ F.y_actual=RY[1]; F.x_actual=RX[1]; F.FSTATE=F_SECOND; Laser_State=ANY; } } }
    } else if(Laser_State === START_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[1])<TH_NON){ F.x_actual=RX[1]; F.y_actual=RY[1]; F.FSTATE=F_SECOND; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[1])<TH_NON){ F.x_actual=RX[1]; F.y_actual=RY[1]; F.FSTATE=F_SECOND; Laser_State=ANY; } } }
    } else if(Laser_State === SECOND_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[2])<TH_NON){ F.y_actual=RY[2]; F.x_actual=RX[2]; F.FSTATE=F_THRID; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[2])<TH_NON){ F.y_actual=RY[2]; F.x_actual=RX[2]; F.FSTATE=F_THRID; Laser_State=ANY; } } }
    } else if(Laser_State === SECOND_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[2])<TH_NON){ F.x_actual=RX[2]; F.y_actual=RY[2]; F.FSTATE=F_THRID; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[2])<TH_NON){ F.x_actual=RX[2]; F.y_actual=RY[2]; F.FSTATE=F_THRID; Laser_State=ANY; } } }
    } else if(Laser_State === THRID_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[3])<TH_NON){ F.y_actual=RY[3]; F.x_actual=RX[3]; F.FSTATE=F_FOURTH; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[3])<TH_NON){ F.y_actual=RY[3]; F.x_actual=RX[3]; F.FSTATE=F_FOURTH; Laser_State=ANY; } } }
    } else if(Laser_State === THRID_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[3])<TH_NON){ F.x_actual=RX[3]; F.y_actual=RY[3]; F.FSTATE=F_FOURTH; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[3])<TH_NON){ F.x_actual=RX[3]; F.y_actual=RY[3]; F.FSTATE=F_FOURTH; Laser_State=ANY; } } }
    } else if(Laser_State === FOURTH_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet);
        if(myabs(F.x_actual-RX[0])<TH_FOURTH){ F.y_actual=RY[0]; F.x_actual=RX[0]; F.X_AXIS=F.x_actual; F.Y_AXIS=F.y_actual; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet);
        if(myabs(F.x_actual-RX[0])<TH_FOURTH){ F.y_actual=RY[0]; F.x_actual=RX[0]; F.X_AXIS=F.x_actual; F.Y_AXIS=F.y_actual; } } }
    } else if(Laser_State === FOURTH_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[0])<TH_FOURTH){ F.x_actual=RX[0]; F.y_actual=RY[0]; F.X_AXIS=F.x_actual; F.Y_AXIS=F.y_actual; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[0])<TH_FOURTH){ F.x_actual=RX[0]; F.y_actual=RY[0]; F.X_AXIS=F.x_actual; F.Y_AXIS=F.y_actual; } } }
    }

    path.push([F.x_actual, F.y_actual]);

    // 完成判定：进入第四段且首次抵达起点角
    if(doneTick < 0 && F.FSTATE === F_FOURTH &&
       myabs(F.x_actual-RX[0]) < TH_NON && myabs(F.y_actual-RY[0]) < TH_NON){
      doneTick = tick;
    }
  }
  return { path, doneTick, finalF:F };
}

/* =============================================================================
 * 仿真版 Motion_TarCtrl_Black  —— 对应固件 Motion_TarCtrl_Black（A4 黑胶带框）
 * 与固件一致：
 *   - Box_Square_State 含 #if BLACK_FRAME_START_FROM_CORNER 分支（JS: startFromCorner）：
 *       角点起步 → x_actual=corner0、FSTATE=Start_To_Second（规避 BUG3）；
 *       否则     → FSTATE=Centy_To_Start（主流程）。
 *   - Centy_To_Start 起点取【黑框自身四角几何中心 bx_c/by_c】（BUG2 修复），非红框标定中心。
 *   - 完成阈值全程 <2（与 contorl.c 中 BLACK_FRAME_STEP=1 路径一致）。
 * ============================================================================= */
function Motion_TarCtrl_Black_Sim(RX, RY, step, startFromCorner, bugMode, maxTicks){
  // 黑框中心：BUG2 修复后使用自身四角几何中心 bx_c/by_c（Centy_To_Start 分支内部计算）
  const bx_c = (RX[0]+RX[1]+RX[2]+RX[3]) / 4;
  const by_c = (RY[0]+RY[1]+RY[2]+RY[3]) / 4;
  const F = {
    x_centry:bx_c, y_centry:by_c,
    FSTATE: startFromCorner ? F_START : F_CENTRY,   // #if BLACK_FRAME_START_FROM_CORNER
    x_actual: startFromCorner ? RX[0] : bx_c,
    y_actual: startFromCorner ? RY[0] : by_c,
    Slope:0, Intercpet:0,
    fixed_x:null, dx:0, dy:0, Is_10ms_YES:1,
    X_AXIS:0, Y_AXIS:0
  };
  let Laser_State = BOX;
  const path = [];
  let doneTick = -1;
  const TH = 2;   // 黑框所有段完成阈值均 <2（与 contorl.c BLACK_FRAME_STEP=1 一致）

  function line_x(y){
    if(!bugMode && F.fixed_x !== null) return F.fixed_x;   // 修正版：竖边保持 x 常量
    return calcX(y, F.Slope, F.Intercpet);                 // 旧版：竖边 (y-int)/inf ≈ 0
  }

  for(let tick=0; tick<maxTicks; tick++){
    F.Is_10ms_YES = 1;

    if(Laser_State === BOX){
      Laser_State = ANY;
      if(startFromCorner){   // 备选方案（规避 BUG3）：直接从胶带角点[0]起步，跳过"中心→角点"脱胶段
        F.x_actual = RX[0];
        F.y_actual = RY[0];
        F.FSTATE   = F_START;
      } else {
        F.FSTATE   = F_CENTRY;
      }
    } else if(Laser_State === ANY){
      if(F.FSTATE === F_CENTRY){
        // 修复 BUG2：使用黑框自身四角几何中心 bx_c/by_c
        F.Slope = calcSlope(bx_c,by_c,RX[0],RY[0]);
        F.fixed_x = (F.Slope===Infinity)? bx_c : null;
        F.Intercpet = calcInt(bx_c,by_c,F.Slope);
        F.dx = RX[0]-bx_c; F.dy = RY[0]-by_c;
        Laser_State = CROSS;
      } else if(F.FSTATE === F_START){
        F.Slope = calcSlope(RX[0],RY[0],RX[1],RY[1]);
        F.fixed_x = (F.Slope===Infinity)? RX[0] : null;
        F.Intercpet = calcInt(RX[0],RY[0],F.Slope);
        F.dx = RX[1]-RX[0]; F.dy = RY[1]-RY[0];
        Laser_State = CROSS;
      } else if(F.FSTATE === F_SECOND){
        F.Slope = calcSlope(RX[1],RY[1],RX[2],RY[2]);
        F.fixed_x = (F.Slope===Infinity)? RX[1] : null;
        F.Intercpet = calcInt(RX[1],RY[1],F.Slope);
        F.dx = RX[2]-RX[1]; F.dy = RY[2]-RY[1];
        Laser_State = CROSS;
      } else if(F.FSTATE === F_THRID){
        F.Slope = calcSlope(RX[2],RY[2],RX[3],RY[3]);
        F.fixed_x = (F.Slope===Infinity)? RX[2] : null;
        F.Intercpet = calcInt(RX[2],RY[2],F.Slope);
        F.dx = RX[3]-RX[2]; F.dy = RY[3]-RY[2];
        Laser_State = CROSS;
      } else if(F.FSTATE === F_FOURTH){
        F.Slope = calcSlope(RX[3],RY[3],RX[0],RY[0]);
        F.fixed_x = (F.Slope===Infinity)? RX[3] : null;
        F.Intercpet = calcInt(RX[3],RY[3],F.Slope);
        F.dx = RX[0]-RX[3]; F.dy = RY[0]-RY[3];
        Laser_State = CROSS;
      }
    } else if(Laser_State === CROSS){
      if(F.FSTATE === F_CENTRY){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=bx_c; F.y_actual=by_c; Laser_State=CENTRY_X; }
        else { F.x_actual=bx_c; F.y_actual=by_c; Laser_State=CENTRY_Y; }
      } else if(F.FSTATE === F_START){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=RX[0]; F.y_actual=RY[0]; Laser_State=START_X; }
        else { F.y_actual=RY[0]; F.x_actual=RX[0]; Laser_State=START_Y; }
      } else if(F.FSTATE === F_SECOND){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=RX[1]; F.y_actual=RY[1]; Laser_State=SECOND_X; }
        else { F.x_actual=RX[1]; F.y_actual=RY[1]; Laser_State=SECOND_Y; }
      } else if(F.FSTATE === F_THRID){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=RX[2]; F.y_actual=RY[2]; Laser_State=THRID_X; }
        else { F.x_actual=RX[2]; F.y_actual=RY[2]; Laser_State=THRID_Y; }
      } else if(F.FSTATE === F_FOURTH){
        if(myabs(F.dx)-myabs(F.dy) > 0){ F.x_actual=RX[3]; F.y_actual=RY[3]; Laser_State=FOURTH_X; }
        else { F.x_actual=RX[3]; F.y_actual=RY[3]; Laser_State=FOURTH_Y; }
      }
    } else if(Laser_State === CENTRY_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[0])<TH){ F.y_actual=RY[0]; F.x_actual=RX[0]; F.FSTATE=F_START; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[0])<TH){ F.y_actual=RY[0]; F.x_actual=RX[0]; F.FSTATE=F_START; Laser_State=ANY; } } }
    } else if(Laser_State === CENTRY_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step;
        if(myabs(F.y_actual-RY[0])<TH){ F.x_actual=RX[0]; F.y_actual=RY[0]; F.FSTATE=F_START; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step;
        if(myabs(F.y_actual-RY[0])<TH){ F.x_actual=RX[0]; F.y_actual=RY[0]; F.FSTATE=F_START; Laser_State=ANY; } } }
    } else if(Laser_State === START_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[1])<TH){ F.y_actual=RY[1]; F.x_actual=RX[1]; F.FSTATE=F_SECOND; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[1])<TH){ F.y_actual=RY[1]; F.x_actual=RX[1]; F.FSTATE=F_SECOND; Laser_State=ANY; } } }
    } else if(Laser_State === START_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[1])<TH){ F.x_actual=RX[1]; F.y_actual=RY[1]; F.FSTATE=F_SECOND; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[1])<TH){ F.x_actual=RX[1]; F.y_actual=RY[1]; F.FSTATE=F_SECOND; Laser_State=ANY; } } }
    } else if(Laser_State === SECOND_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[2])<TH){ F.y_actual=RY[2]; F.x_actual=RX[2]; F.FSTATE=F_THRID; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[2])<TH){ F.y_actual=RY[2]; F.x_actual=RX[2]; F.FSTATE=F_THRID; Laser_State=ANY; } } }
    } else if(Laser_State === SECOND_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[2])<TH){ F.x_actual=RX[2]; F.y_actual=RY[2]; F.FSTATE=F_THRID; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[2])<TH){ F.x_actual=RX[2]; F.y_actual=RY[2]; F.FSTATE=F_THRID; Laser_State=ANY; } } }
    } else if(Laser_State === THRID_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[3])<TH){ F.y_actual=RY[3]; F.x_actual=RX[3]; F.FSTATE=F_FOURTH; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet); F.Is_10ms_YES=0;
        if(myabs(F.x_actual-RX[3])<TH){ F.y_actual=RY[3]; F.x_actual=RX[3]; F.FSTATE=F_FOURTH; Laser_State=ANY; } } }
    } else if(Laser_State === THRID_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[3])<TH){ F.x_actual=RX[3]; F.y_actual=RY[3]; F.FSTATE=F_FOURTH; Laser_State=ANY; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[3])<TH){ F.x_actual=RX[3]; F.y_actual=RY[3]; F.FSTATE=F_FOURTH; Laser_State=ANY; } } }
    } else if(Laser_State === FOURTH_X){
      if(F.dx>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.x_actual+=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet);
        if(myabs(F.x_actual-RX[0])<TH){ F.y_actual=RY[0]; F.x_actual=RX[0]; F.X_AXIS=F.x_actual; F.Y_AXIS=F.y_actual; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.x_actual-=step; F.y_actual=calcY(F.x_actual,F.Slope,F.Intercpet);
        if(myabs(F.x_actual-RX[0])<TH){ F.y_actual=RY[0]; F.x_actual=RX[0]; F.X_AXIS=F.x_actual; F.Y_AXIS=F.y_actual; } } }
    } else if(Laser_State === FOURTH_Y){
      if(F.dy>0){ if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual+=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[0])<TH){ F.x_actual=RX[0]; F.y_actual=RY[0]; F.X_AXIS=F.x_actual; F.Y_AXIS=F.y_actual; } } }
      else { if(F.Is_10ms_YES===1){ F.Is_10ms_YES=0; F.y_actual-=step; F.x_actual=line_x(F.y_actual);
        if(myabs(F.y_actual-RY[0])<TH){ F.x_actual=RX[0]; F.y_actual=RY[0]; F.X_AXIS=F.x_actual; F.Y_AXIS=F.y_actual; } } }
    }

    path.push([F.x_actual, F.y_actual]);

    // 完成判定：进入第四段且首次抵达起点角
    if(doneTick < 0 && F.FSTATE === F_FOURTH &&
       myabs(F.x_actual-RX[0]) < TH && myabs(F.y_actual-RY[0]) < TH){
      doneTick = tick;
    }
  }
  return { path, doneTick, finalF:F };
}

/* ---------------- 自测：验证三场景逻辑不破坏 ---------------- */
if(typeof require !== 'undefined' && require.main === module){
  const PX_PER_M = 352.0;
  const TAPE_HALF = 0.009 * PX_PER_M;   // 1.8cm 胶带半宽(px)
  function pointToSeg(px,py,ax,ay,bx,by){
    const dx=bx-ax, dy=by-ay, l2=dx*dx+dy*dy;
    if(l2<1e-9) return Math.hypot(px-ax,py-ay);
    let t=((px-ax)*dx+(py-ay)*dy)/l2; t=Math.max(0,Math.min(1,t));
    return Math.hypot(px-(ax+t*dx), py-(ay+t*dy));
  }
  function distToBorder(px,py,RX,RY){
    let m=Infinity;
    for(let i=0;i<4;i++){ const a=i,b=(i+1)%4; m=Math.min(m, pointToSeg(px,py,RX[a],RY[a],RX[b],RY[b])); }
    return m;
  }
  function offMaxCm(path, RX, RY){
    let max=0, run=0;
    for(let i=1;i<path.length;i++){
      const seg=Math.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])/PX_PER_M;
      if(distToBorder(path[i][0],path[i][1],RX,RY) > TAPE_HALF){ run+=seg; max=Math.max(max,run); } else { run=0; }
    }
    return max*100; // cm
  }

  // 红大方框（标定矩形 0.5×0.5m，步进 7）
  const big = Motion_TarCtrl_Sim([32,208,208,32],[7,7,183,183],120,95,7,false,6000);
  console.log('[BIG ] path=%d doneTick=%s', big.path.length, big.doneTick);

  // A4 标准位（210×297mm → 74×105px，步进 1）
  const a4 = [83,157,157,83], a4y = [43,43,148,148];
  const frame = Motion_TarCtrl_Black_Sim(a4, a4y, 1, false, false, 6000);   // 主流程：中心起步（BUG3 段）
  const corner = Motion_TarCtrl_Black_Sim(a4, a4y, 1, true,  false, 6000);   // 备选：角点起步
  console.log('[A4 frame ] path=%d doneTick=%s maxOff=%scm', frame.path.length, frame.doneTick, offMaxCm(frame.path,a4,a4y).toFixed(1));
  console.log('[A4 corner] path=%d doneTick=%s maxOff=%scm', corner.path.length, corner.doneTick, offMaxCm(corner.path,a4,a4y).toFixed(1));

  const ok = big.doneTick>=0 && frame.doneTick>=0 && corner.doneTick>=0 &&
             offMaxCm(frame.path,a4,a4y) > 5 && offMaxCm(corner.path,a4,a4y) < 0.5;
  console.log(ok ? 'PASS: 仿真两函数逻辑与固件预期一致（中心起步有脱胶段、角点起步全程贴胶带）'
                 : 'FAIL: 逻辑与预期不符，请检查');
}
