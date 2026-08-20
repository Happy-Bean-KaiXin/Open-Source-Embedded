/* =============================================================================
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

计算一次函数的斜率
float calculateSlope(float x1, float y1, float x2, float y2) {
	// [BUGFIX] 竖边(x2==x1)时斜率无意义，直接返回 0：
	//   该分段由各 *_Y_* 分支 if(Flag.dx!=0) 守卫保持 x 为常量，
	//   避免除零产生 inf/未定义行为（与 Motion_TarCtrl_Sim.js 的 fixed_x 等价）。
	if ((x2 - x1) == 0.0f) return 0.0f;
    return ((y2 - y1) / (x2 - x1));
}
计算一次函数的截距 其中slope是斜率
int calculateIntercept(int x1, int y1, float slope) {
    return (y1 - slope * x1);
}
计算 y 值  intercept是截距   传入x的值计算y值
int calculateY(int x, float slope, int intercept) {
    return slope * x + intercept;
}
计算 x 值  传入y计算x的值
int calculateX(float y, float slope, float intercept) {
	// [BUGFIX] Div-by-zero guard: horizontal lines (slope=0) cause HardFault.
	//   Return 0 as safe fallback — caller should use calculateY for horizontal edges.
	if(slope == 0.0f) return 0;
	return (y - intercept) / slope;
}
运动目标控制处理函数
void Motion_TarCtrl(int* RetangleX, int* RetangleY) {
	switch(RED_LASER.Laser_State) {
		case Box_Square_State:
				RED_LASER.Laser_State = Any_Rectang_Box_State;
			
			break;
		case Any_Rectang_Box_State:   // 任意位置起点状态，从起点到达终点（下一个起点），第一步计算斜率，截距
			/********************************** 状态1 ***********************************************/
			if(Flag.FSTATE == Centy_To_Start) {  // 状态1
					Flag.Slope = calculateSlope(Flag.x_centry, Flag.y_centry, RetangleX[0], RetangleY[0]);  // 传入中心坐标，和任意方框的起点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(Flag.x_centry, Flag.y_centry, Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (RetangleX[0] - Flag.x_centry);
					Flag.dy = (RetangleY[0] - Flag.y_centry);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			/********************************** 状态2 ***********************************************/
			else if(Flag.FSTATE == Start_To_Second) {  
					Flag.Slope = calculateSlope(RetangleX[0], RetangleY[0], RetangleX[1], RetangleY[1]);  // 传入起点坐标，终点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(RetangleX[0], RetangleY[0], Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (RetangleX[1] - RetangleX[0]);
					Flag.dy = (RetangleY[1] - RetangleY[0]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			/********************************** 状态3 ***********************************************/
			else if(Flag.FSTATE == Second_To_Thrid) {  // 状态3
					Flag.Slope = calculateSlope(RetangleX[1], RetangleY[1], RetangleX[2], RetangleY[2]);  // 传入起点坐标，终点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(RetangleX[1], RetangleY[1], Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (RetangleX[2] - RetangleX[1]);
					Flag.dy = (RetangleY[2] - RetangleY[1]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			/********************************** 状态4 ***********************************************/
			else if(Flag.FSTATE == Thrid_To_Fourth) {  // 状态4
					Flag.Slope = calculateSlope(RetangleX[2], RetangleY[2], RetangleX[3], RetangleY[3]);  // 传入起点坐标，终点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(RetangleX[2], RetangleY[2], Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (RetangleX[3] - RetangleX[2]);
					Flag.dy = (RetangleY[3] - RetangleY[2]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			/********************************** 状态5 ***********************************************/
			else if(Flag.FSTATE == Fourth_To_End) {  // 状态5
					Flag.Slope = calculateSlope(RetangleX[3], RetangleY[3], RetangleX[0], RetangleY[0]);  // 传入起点坐标，终点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(RetangleX[3], RetangleY[3], Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (RetangleX[0] - RetangleX[3]);
					Flag.dy = (RetangleY[0] - RetangleY[3]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			break;
		/********************************** 等待状态1-2-3-4-5所进入状态 ***********************************************/
		case Cross_State:
			/********************************** 状态1的等待状态 ***********************************************/
			if(Flag.FSTATE == Centy_To_Start) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = Flag.x_centry;  // 让实际值y等于y的中心坐标
					Flag.y_actual = Flag.x_centry;
					RED_LASER.Laser_State = Centry_X_Start_State;   // 以x为变量算y
				}
				else {                                            // 如果dy大于dx
					Flag.x_actual = Flag.x_centry;  // 让实际值y等于y的中心坐标
					Flag.y_actual = Flag.x_centry;
					RED_LASER.Laser_State = Centry_Y_Start_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			/********************************** 状态2的等待状态 ***********************************************/
			else if(Flag.FSTATE == Start_To_Second) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = RetangleX[0];   // 让实际值x  方框起点坐标x
					Flag.y_actual = RetangleY[0];   // 方框起始坐标y

					RED_LASER.Laser_State = Start_X_Second_State;   // 以x为变量算y
				}
				else {                                            // 如果dy大于dx
					Flag.y_actual = RetangleY[0];   // 					
					Flag.x_actual = RetangleY[0];  
					RED_LASER.Laser_State = Start_Y_Second_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			/********************************** 状态3的等待状态 ***********************************************/
			else if(Flag.FSTATE == Second_To_Thrid) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = RetangleX[1];   // 让实际值x  方框第二点坐标x
					Flag.y_actual = RetangleY[1];   // 方框第二点坐标y
					RED_LASER.Laser_State = Second_X_Thrid_State;   // 进入下一个状态
				}
				else {                                            // 如果dy大于dx
					Flag.x_actual = RetangleX[1];   // 让实际值x  方框第二点坐标x
					Flag.y_actual = RetangleY[1];   // 方框第二点坐标y
					RED_LASER.Laser_State = Second_Y_Thrid_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			/********************************** 状态4的等待状态 ***********************************************/
			else if(Flag.FSTATE == Thrid_To_Fourth) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = RetangleX[2];   // 让实际值x  方框第三点坐标x
					Flag.y_actual = RetangleY[2];   // 方框第三点坐标y
					RED_LASER.Laser_State = Thrid_X_Fourth_State;   // 以x为变量算y
				}
				else {                                            // 如果dy大于dx
					Flag.x_actual = RetangleX[2];   // 让实际值x  方框第三点坐标x
					Flag.y_actual = RetangleY[2];   // 方框第三点坐标y
					RED_LASER.Laser_State = Thrid_Y_Fourth_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			/********************************** 状态5的等待状态 ***********************************************/
			else if(Flag.FSTATE == Fourth_To_End) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = RetangleX[3];   // 让实际值x  方框第四点坐标x
					Flag.y_actual = RetangleY[3];   // 方框第四点坐标y
					RED_LASER.Laser_State = Fourth_X_End_State;   // 以x为变量算y
				}
				else {                                            // 如果dy大于dx
					Flag.x_actual = RetangleX[3];   // 让实际值x  方框第四点坐标x
					Flag.y_actual = RetangleY[3];   // 方框第四点坐标y
					RED_LASER.Laser_State = Fourth_Y_End_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			break;
		/********************************** 状态1下的子状态 ***********************************************/
		case Centry_X_Start_State:        // 以x为变量算y的状态  中心到方框起点使用x当变量的状态
//			Flag.x_actual = Flag.x_centry;  // 让实际值y等于y的中心坐标
//			Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);  // 使用x的实际坐标值算y
			
			if(Flag.dx > 0) {              // 如果直线方程的dx>0 就让x的实际值加加
				if(Flag.Is_10ms_YES == 1) {  // 如果10ms到了，就让实际值加1
					
					Flag.x_actual+=7;
					Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);  // 使用x的实际坐标值算y
					Flag.Is_10ms_YES = 0;
					if(myabs(Flag.x_actual - RetangleX[0]) < 4) {  // 如果实际值等于到达的第一个目标值，进行下一个状态
						Flag.y_actual = RetangleY[0];      // 让Y的实际值强行等于目标值
						Flag.x_actual = RetangleX[0];
						// 进入下一个状态
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
			}
			else {                         // 如果dx<0 就让实际值减减
				if(Flag.Is_10ms_YES == 1) {  // 就让实际值减1
					
					Flag.x_actual-=7;
					Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet); 
					Flag.Is_10ms_YES = 0;
					if(myabs(Flag.x_actual - RetangleX[0]) < 4) {  // 如果实际值等于到达的第一个目标值，进行下一个状态
						Flag.y_actual = RetangleY[0];      // 让Y的实际值强行等于目标值
						Flag.x_actual = RetangleX[0];
						// 进入下一个状态
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
				
			}
			

			break;
		case Centry_Y_Start_State:   // 使用y当变量的状态
//			Flag.y_actual = Flag.y_centry;   // 让实际值y等于y的中心坐标
//			Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);  // 根据y的实际值求x
			
			if(Flag.dy > 0) {
				if(Flag.Is_10ms_YES == 1) {
					Flag.Is_10ms_YES = 0;
					Flag.y_actual+=7;
					if(myabs(Flag.y_actual - RetangleY[0]) < 4) {
						Flag.x_actual = RetangleX[0];
						Flag.y_actual = RetangleY[0];
						// 进入下一个状态
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
			}
			else {
				if(Flag.Is_10ms_YES == 1) {
					Flag.Is_10ms_YES = 0;
					Flag.y_actual-=7;
					// [BUGFIX] dy<0 分支死区从 <1 修正为 <4，与 dy>0 分支对称 (对齐仿真基线)
					if(myabs(Flag.y_actual - RetangleY[0]) < 4) {
						Flag.x_actual = RetangleX[0];
						Flag.y_actual = RetangleY[0];
						// 进入下一个状态
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}				
			}
			

			break;
			/********************************** 状态2下的子状态 ***********************************************/
			case Start_X_Second_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[1]) < 4) {
							Flag.y_actual = RetangleY[1];
							Flag.x_actual = RetangleX[1];
							// 进入下一个状态
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual-=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[1]) < 4) {
							Flag.y_actual = RetangleY[1];
							Flag.x_actual = RetangleX[1];
							// 进入下一个状态
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				
				
				break;
			case Start_Y_Second_State:
				if(Flag.dy > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual+=7;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - RetangleY[1]) < 4) {
							Flag.x_actual = RetangleX[1];
							Flag.y_actual = RetangleY[1];
							// 进入下一个状态
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual-=7;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - RetangleY[1]) < 4) {
							Flag.x_actual = RetangleX[1];
							Flag.y_actual = RetangleY[1];
							// 进入下一个状态
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				
				break;
			/********************************** 状态3下的子状态 ***********************************************/
			case Second_X_Thrid_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[2]) < 4) {
							Flag.y_actual = RetangleY[2];
							Flag.x_actual = RetangleX[2];
							// 进入下一个状态
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual-=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[2]) < 4) {
							Flag.y_actual = RetangleY[2];
							Flag.x_actual = RetangleX[2];
							// 进入下一个状态
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}				
				break;
			case Second_Y_Thrid_State:
				if(Flag.dy > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual+=7;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - RetangleY[2]) < 4) {
							Flag.x_actual = RetangleX[2];
							Flag.y_actual = RetangleY[2];
							// 进入下一个状态
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual-=7;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - RetangleY[2]) < 4) {
							Flag.x_actual = RetangleX[2];
							Flag.y_actual = RetangleY[2];
							// 进入下一个状态
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				break;
			/********************************** 状态4下的子状态 ***********************************************/
			case Thrid_X_Fourth_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[3]) < 10) {
							Flag.y_actual = RetangleY[3];
							Flag.x_actual = RetangleX[3];
							// 进入下一个状态
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual-=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[3]) < 10) {
							Flag.y_actual = RetangleY[3];
							Flag.x_actual = RetangleX[3];
							// 进入下一个状态
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}	
				break;
			case Thrid_Y_Fourth_State:
				if(Flag.dy > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual+=7;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - RetangleY[3]) < 10) {
							Flag.x_actual = RetangleX[3];
							Flag.y_actual = RetangleY[3];
							// 进入下一个状态
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual-=7;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - RetangleY[3]) < 10) {
							Flag.x_actual = RetangleX[3];
							Flag.y_actual = RetangleY[3];
							// 进入下一个状态
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				break;
			/********************************** 状态5下的子状态 ***********************************************/
			case Fourth_X_End_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[0]) < 10) {
							Flag.y_actual = RetangleY[0];
							Flag.x_actual = RetangleX[0];
							Flag.X_AXIS = Flag.x_actual;
							Flag.Y_AXIS = Flag.y_actual;
//							while((Flag.x_actual == RetangleX[0]) &&(Flag.y_actual == RetangleY[0])) {
//								Flag.Is_Angle_Set = 0;
//								break;
//							}
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual-=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[0]) < 10) {
							Flag.y_actual = RetangleY[0];
							Flag.x_actual = RetangleX[0];
							Flag.X_AXIS = Flag.x_actual;
							Flag.Y_AXIS = Flag.y_actual;
//							while((Flag.x_actual == RetangleX[0]) &&(Flag.y_actual == RetangleY[0])) {
//								Flag.Is_Angle_Set = 0;
//								break;
//							}							
						}
					}				
				}					
				break;
			case Fourth_Y_End_State:
				if(Flag.dy > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual+=7;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - RetangleY[0]) < 	10) {
							Flag.x_actual = RetangleX[0];
							Flag.y_actual = RetangleY[0];
							Flag.X_AXIS = Flag.x_actual;
							Flag.Y_AXIS = Flag.y_actual;
							// 进入下一个状态
//							while((Flag.x_actual == RetangleX[0]) &&(Flag.y_actual == RetangleY[0])) {
//								Flag.Is_Angle_Set = 0;
//								break;
//							}
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual-=7;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - RetangleY[0]) < 10) {
							Flag.x_actual = RetangleX[0];
							Flag.y_actual = RetangleY[0];
							Flag.X_AXIS = Flag.x_actual;
							Flag.Y_AXIS = Flag.y_actual;
							// 进入下一个状态
//							while((Flag.x_actual == RetangleX[0]) &&(Flag.y_actual == RetangleY[0])) {
//								Flag.Is_Angle_Set = 0;
//								break;
//							}
						}
					}				
				}
				break;
		default: break;
	}

}
void Motion_TarCtrl_Black(int* Black_Retanx, int* Black_Retany) {
	switch(RED_LASER.Laser_State) {
		case Box_Square_State:
				RED_LASER.Laser_State = Any_Rectang_Box_State;
			
			break;
		case Any_Rectang_Box_State:   // 任意位置起点状态，从起点到达终点（下一个起点），第一步计算斜率，截距
			/********************************** 状态1 ***********************************************/
			if(Flag.FSTATE == Centy_To_Start) {  // 状态1
					Flag.Slope = calculateSlope(Flag.x_centry, Flag.y_centry, Black_Retanx[0], Black_Retany[0]);  // 传入中心坐标，和任意方框的起点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(Flag.x_centry, Flag.y_centry, Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (Black_Retanx[0] - Flag.x_centry);
					Flag.dy = (Black_Retany[0] - Flag.y_centry);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			/********************************** 状态2 ***********************************************/
			else if(Flag.FSTATE == Start_To_Second) {  
					Flag.Slope = calculateSlope(Black_Retanx[0], Black_Retany[0], Black_Retanx[1], Black_Retany[1]);  // 传入起点坐标，终点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(Black_Retanx[0], Black_Retany[0], Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (Black_Retanx[1] - Black_Retanx[0]);
					Flag.dy = (Black_Retany[1] - Black_Retany[0]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			/********************************** 状态3 ***********************************************/
			else if(Flag.FSTATE == Second_To_Thrid) {  // 状态3
					Flag.Slope = calculateSlope(Black_Retanx[1], Black_Retany[1], Black_Retanx[2], Black_Retany[2]);  // 传入起点坐标，终点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(Black_Retanx[1], Black_Retany[1], Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (Black_Retanx[2] - Black_Retanx[1]);
					Flag.dy = (Black_Retany[2] - Black_Retany[1]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			/********************************** 状态4 ***********************************************/
			else if(Flag.FSTATE == Thrid_To_Fourth) {  // 状态4
					Flag.Slope = calculateSlope(Black_Retanx[2], Black_Retany[2], Black_Retanx[3], Black_Retany[3]);  // 传入起点坐标，终点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(Black_Retanx[2], Black_Retany[2], Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (Black_Retanx[3] - Black_Retanx[2]);
					Flag.dy = (Black_Retany[3] - Black_Retany[2]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			/********************************** 状态5 ***********************************************/
			else if(Flag.FSTATE == Fourth_To_End) {  // 状态5
					Flag.Slope = calculateSlope(Black_Retanx[3], Black_Retany[3], Black_Retanx[0], Black_Retany[0]);  // 传入起点坐标，终点坐标 计算斜率
					Flag.Intercpet = calculateIntercept(Black_Retanx[3], Black_Retany[3], Flag.Slope);          // 计算截距
					// 计算dx， dy
					Flag.dx = (Black_Retanx[0] - Black_Retanx[3]);
					Flag.dy = (Black_Retany[0] - Black_Retany[3]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
						RED_LASER.Laser_State = Cross_State;   // 以x为变量算y
					}
					else {                                            // 如果dy大于dx
						RED_LASER.Laser_State = Cross_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
					}
			}
			break;
		/********************************** 等待状态1-2-3-4-5所进入状态 ***********************************************/
		case Cross_State:
			/********************************** 状态1的等待状态 ***********************************************/
			if(Flag.FSTATE == Centy_To_Start) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = Flag.x_centry;  // 让实际值y等于y的中心坐标
					Flag.y_actual = Flag.x_centry;
					RED_LASER.Laser_State = Centry_X_Start_State;   // 以x为变量算y
				}
				else {                                            // 如果dy大于dx
					Flag.x_actual = Flag.x_centry;  // 让实际值y等于y的中心坐标
					Flag.y_actual = Flag.x_centry;
					RED_LASER.Laser_State = Centry_Y_Start_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			/********************************** 状态2的等待状态 ***********************************************/
			else if(Flag.FSTATE == Start_To_Second) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = Black_Retanx[0];   // 让实际值x  方框起点坐标x
					Flag.y_actual = Black_Retany[0];   // 方框起始坐标y

					RED_LASER.Laser_State = Start_X_Second_State;   // 以x为变量算y
				}
				else {                                            // 如果dy大于dx
					Flag.y_actual = Black_Retanx[0];   // 					
					Flag.x_actual = Black_Retany[0];  
					RED_LASER.Laser_State = Start_Y_Second_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			/********************************** 状态3的等待状态 ***********************************************/
			else if(Flag.FSTATE == Second_To_Thrid) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = Black_Retanx[1];   // 让实际值x  方框第二点坐标x
					Flag.y_actual = Black_Retany[1];   // 方框第二点坐标y
					RED_LASER.Laser_State = Second_X_Thrid_State;   // 进入下一个状态
				}
				else {                                            // 如果dy大于dx
					Flag.x_actual = Black_Retanx[1];   // 让实际值x  方框第二点坐标x
					Flag.y_actual = Black_Retany[1];   // 方框第二点坐标y
					RED_LASER.Laser_State = Second_Y_Thrid_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			/********************************** 状态4的等待状态 ***********************************************/
			else if(Flag.FSTATE == Thrid_To_Fourth) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = Black_Retanx[2];   // 让实际值x  方框第三点坐标x
					Flag.y_actual = Black_Retany[2];   // 方框第三点坐标y
					RED_LASER.Laser_State = Thrid_X_Fourth_State;   // 以x为变量算y
				}
				else {                                            // 如果dy大于dx
					Flag.x_actual = Black_Retanx[2];   // 让实际值x  方框第三点坐标x
					Flag.y_actual = Black_Retany[2];   // 方框第三点坐标y
					RED_LASER.Laser_State = Thrid_Y_Fourth_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			/********************************** 状态5的等待状态 ***********************************************/
			else if(Flag.FSTATE == Fourth_To_End) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // 比较dx与dy的大小去顶x与y谁是变量
					Flag.x_actual = Black_Retanx[3];   // 让实际值x  方框第四点坐标x
					Flag.y_actual = Black_Retany[3];   // 方框第四点坐标y
					RED_LASER.Laser_State = Fourth_X_End_State;   // 以x为变量算y
				}
				else {                                            // 如果dy大于dx
					Flag.x_actual = Black_Retanx[3];   // 让实际值x  方框第四点坐标x
					Flag.y_actual = Black_Retany[3];   // 方框第四点坐标y
					RED_LASER.Laser_State = Fourth_Y_End_State;   // 则以y为变量算x，进入以y变量算x的状态（中心到方框起点使用y当变量的状态）
				}
			}
			break;
		/********************************** 状态1下的子状态 ***********************************************/
		case Centry_X_Start_State:        // 以x为变量算y的状态  中心到方框起点使用x当变量的状态
//			Flag.x_actual = Flag.x_centry;  // 让实际值y等于y的中心坐标
//			Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);  // 使用x的实际坐标值算y
			
			if(Flag.dx > 0) {              // 如果直线方程的dx>0 就让x的实际值加加
				if(Flag.Is_10ms_YES == 1) {  // 如果10ms到了，就让实际值加1
					
					Flag.x_actual+=1;
					Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);  // 使用x的实际坐标值算y
					Flag.Is_10ms_YES = 0;
					if(myabs(Flag.x_actual - Black_Retanx[0]) < 2) {  // 如果实际值等于到达的第一个目标值，进行下一个状态
						Flag.y_actual = Black_Retany[0];      // 让Y的实际值强行等于目标值
						Flag.x_actual = Black_Retanx[0];
						// 进入下一个状态
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
			}
			else {                         // 如果dx<0 就让实际值减减
				if(Flag.Is_10ms_YES == 1) {  // 就让实际值减1
					
					Flag.x_actual-=1;
					Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet); 
					Flag.Is_10ms_YES = 0;
					if(myabs(Flag.x_actual - Black_Retanx[0]) < 2) {  // 如果实际值等于到达的第一个目标值，进行下一个状态
						Flag.y_actual = Black_Retany[0];      // 让Y的实际值强行等于目标值
						Flag.x_actual = Black_Retanx[0];
						// 进入下一个状态
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
				
			}
			

			break;
		case Centry_Y_Start_State:   // 使用y当变量的状态
//			Flag.y_actual = Flag.y_centry;   // 让实际值y等于y的中心坐标
//			Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);  // 根据y的实际值求x
			
			if(Flag.dy > 0) {
				if(Flag.Is_10ms_YES == 1) {
					Flag.Is_10ms_YES = 0;
					Flag.y_actual+=1;
					if(myabs(Flag.y_actual - Black_Retany[0]) < 2) {
						Flag.x_actual = Black_Retanx[0];
						Flag.y_actual = Black_Retany[0];
						// 进入下一个状态
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
			}
			else {
				if(Flag.Is_10ms_YES == 1) {
					Flag.Is_10ms_YES = 0;
					Flag.y_actual-=1;
					if(myabs(Flag.y_actual - Black_Retany[0]) < 2) {
						Flag.x_actual = Black_Retanx[0];
						Flag.y_actual = Black_Retany[0];
						// 进入下一个状态
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}				
			}
			

			break;
			/********************************** 状态2下的子状态 ***********************************************/
			case Start_X_Second_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[1]) < 2) {
							Flag.y_actual = Black_Retany[1];
							Flag.x_actual = Black_Retanx[1];
							// 进入下一个状态
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual-=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[1]) < 2) {
							Flag.y_actual = Black_Retany[1];
							Flag.x_actual = Black_Retanx[1];
							// 进入下一个状态
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				
				
				break;
			case Start_Y_Second_State:
				if(Flag.dy > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual+=7;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - Black_Retany[1]) < 2) {
							Flag.x_actual = Black_Retanx[1];
							Flag.y_actual = Black_Retany[1];
							// 进入下一个状态
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual-=1;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - Black_Retany[1]) < 2) {
							Flag.x_actual = Black_Retanx[1];
							Flag.y_actual = Black_Retany[1];
							// 进入下一个状态
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				
				break;
			/********************************** 状态3下的子状态 ***********************************************/
			case Second_X_Thrid_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[2]) < 2) {
							Flag.y_actual = Black_Retany[2];
							Flag.x_actual = Black_Retanx[2];
							// 进入下一个状态
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual-=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[2]) < 2) {
							Flag.y_actual = Black_Retany[2];
							Flag.x_actual = Black_Retanx[2];
							// 进入下一个状态
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}				
				break;
			case Second_Y_Thrid_State:
				if(Flag.dy > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual+=1;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - Black_Retany[2]) < 2) {
							Flag.x_actual = Black_Retanx[2];
							Flag.y_actual = Black_Retany[2];
							// 进入下一个状态
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual-=1;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - Black_Retany[2]) < 2) {
							Flag.x_actual = Black_Retanx[2];
							Flag.y_actual = Black_Retany[2];
							// 进入下一个状态
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				break;
			/********************************** 状态4下的子状态 ***********************************************/
			case Thrid_X_Fourth_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[3]) < 2) {
							Flag.y_actual = Black_Retany[3];
							Flag.x_actual = Black_Retanx[3];
							// 进入下一个状态
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual-=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[3]) < 2) {
							Flag.y_actual = Black_Retany[3];
							Flag.x_actual = Black_Retanx[3];
							// 进入下一个状态
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}	
				break;
			case Thrid_Y_Fourth_State:
				if(Flag.dy > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual+=1;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - Black_Retany[3]) < 2) {
							Flag.x_actual = Black_Retanx[3];
							Flag.y_actual = Black_Retany[3];
							// 进入下一个状态
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual-=1;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - Black_Retany[3]) < 2) {
							Flag.x_actual = Black_Retanx[3];
							Flag.y_actual = Black_Retany[3];
							// 进入下一个状态
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				break;
			/********************************** 状态5下的子状态 ***********************************************/
			case Fourth_X_End_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[0]) < 2) {
							Flag.y_actual = Black_Retany[0];
							Flag.x_actual = Black_Retanx[0];
							Flag.X_AXIS = Flag.x_actual;
							Flag.Y_AXIS = Flag.y_actual;
//							while((Flag.x_actual == RetangleX[0]) &&(Flag.y_actual == RetangleY[0])) {
//								Flag.Is_Angle_Set = 0;
//								break;
//							}
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual-=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[0]) < 2) {
							Flag.y_actual = Black_Retany[0];
							Flag.x_actual = Black_Retanx[0];
							Flag.X_AXIS = Flag.x_actual;
							Flag.Y_AXIS = Flag.y_actual;
//							while((Flag.x_actual == RetangleX[0]) &&(Flag.y_actual == RetangleY[0])) {
//								Flag.Is_Angle_Set = 0;
//								break;
//							}							
						}
					}				
				}					
				break;
			case Fourth_Y_End_State:
				if(Flag.dy > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual+=1;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - Black_Retany[0]) < 	2) {
							Flag.x_actual = Black_Retanx[0];
							Flag.y_actual = Black_Retany[0];
							Flag.X_AXIS = Flag.x_actual;
							Flag.Y_AXIS = Flag.y_actual;
							// 进入下一个状态
//							while((Flag.x_actual == RetangleX[0]) &&(Flag.y_actual == RetangleY[0])) {
//								Flag.Is_Angle_Set = 0;
//								break;
//							}
						}
					}
				}
				else {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.y_actual-=1;
						Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.y_actual - Black_Retany[0]) < 2) {
							Flag.x_actual = Black_Retanx[0];
							Flag.y_actual = Black_Retany[0];
							Flag.X_AXIS = Flag.x_actual;
							Flag.Y_AXIS = Flag.y_actual;
							// 进入下一个状态
//							while((Flag.x_actual == RetangleX[0]) &&(Flag.y_actual == RetangleY[0])) {
//								Flag.Is_Angle_Set = 0;
//								break;
//							}
						}
					}				
				}
				break;
		default: break;
	}

}
