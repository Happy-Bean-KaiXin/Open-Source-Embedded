#include "contorl.h"

PARAM_t Flag;   // 标志位结构体
LASER_t RED_LASER;  // 激光结构体

float anglex, angley;                  // x轴，y轴的角度值
int RetangleX[4] = {21, 183, 184, 29};                    // 大框x轴的坐标
int RetangleY[4] = {30, 32, 184, 189};                    // 大框y轴的坐标

int Black_Retanx[4] = {77, 149, 149, 84};                                      // 黑色矩形框坐标x
int Black_Retany[4] = {47, 46, 94, };                                      // 黑色矩形框坐标y

int i;

// 定时器回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		
		static int cnt;
    if(htim->Instance == TIM4)//10ms
    {
			if(Flag.Is_Angle_Set) {
				// 使用PID计算角度
				Flag.Anglex = Position_PID_RealizeX(Flag.X_AXIS, Flag.x_actual);
				Flag.Angley = Position_PID_RealizeY(Flag.Y_AXIS, Flag.y_actual);
				
				// 舵机PID
				Servo_Position.Servo_TarYout = Servo_Angle_Compare(Flag.Angley);  // 设置舵机转动的目标值
				Servo_Position.Servo_TarXout = Servo_Angle_Compare(Flag.Anglex);  // 设置舵机转动的目标值
				
				//输入参数：         舵机转动固定角度需要的定时器比较值  舵机转动角度的目标值
				Init_CompareY += Servo_Position_PIDY(Init_CompareY, Servo_Position.Servo_TarYout);  // 位置式PID运算 Init_CompareY当前舵机转动的位置
				angley = Servo_Compare_Angle(Init_CompareY);  // 将PID计算的输出比较值转换为角度值
				
				Init_CompareX += Servo_Position_PIDX(Init_CompareX, Servo_Position.Servo_TarXout);  // 位置式PID1运算 Init_CompareY当前舵机转动的位置
				anglex = Servo_Compare_Angle(Init_CompareX);  // 将PID计算的输出比较值转换为角度值
				
				// 定死角度，根据框的大小定死
				LIMIT(anglex, -90, 90);
				LIMIT(angley, -90, 90);
				
				Servo_X_Angle_Set(anglex);
				Servo_Y_Angle_Set(angley);    // 设置舵机转动的角度
				
				// 判断程序是否运行
				if(++cnt > 50) {
					cnt = 0;
					i++;
				}
			
			}
    }
		else if(htim->Instance == TIM3) {
			Flag.Is_10ms_YES = 1;
						// 判断程序是否运行
		}
}

// 串口发送处理函数
void print(UART_HandleTypeDef* huart, const char* buf, ...) {  // 串口函数
  const char *p = buf;
  char str[255] = {0};
  va_list v;
  va_start(v, buf);
  vsprintf(str, buf, v); //使用可变参数的字符串打印。类似sprintf
  if(HAL_UART_Transmit(huart, (uint8_t* )str, strlen(str), 1000) != HAL_OK) {
		Error_Handler();
	}
  va_end(v);
}

/*******************************************显示处理函数*******************************************/
void OLED_Proc() {
	char buf[22];
	static __IO uint32_t uwTick_OLED_Speed;
	
	if(uwTick - uwTick_OLED_Speed < 100) return;
	uwTick_OLED_Speed = uwTick;
	
	if(Flag.Is_OLED_Face == 0) {
		
		sprintf(buf, "       ");
		OLED_ShowStr(0, 0, (uint8_t* )buf, 1);
		sprintf(buf, "CompareY = %.2f   ", Init_CompareY);
		OLED_ShowStr(0, 1, (uint8_t* )buf, 1);
		sprintf(buf, "TarYCom = %d    ", Servo_Position.Servo_TarYout);
		OLED_ShowStr(0, 2, (uint8_t* )buf, 1);
		sprintf(buf, "i = %d      ", i);
		OLED_ShowStr(0, 3, (uint8_t* )buf, 1);
		sprintf(buf, "angley = %.2f      ", angley);
		OLED_ShowStr(0, 4, (uint8_t* )buf, 1);
		sprintf(buf, "anglex = %.2f  ", anglex);
		OLED_ShowStr(0, 5, (uint8_t* )buf, 1);
		sprintf(buf, "      ");
		OLED_ShowStr(0, 6, (uint8_t* )buf, 1);
		sprintf(buf, "       ");
		OLED_ShowStr(0, 7, (uint8_t* )buf, 1);
	}
	else if(Flag.Is_OLED_Face == 1) {
		sprintf(buf, "x = %03d y = %03d  ", Flag.X_AXIS, Flag.Y_AXIS);
		OLED_ShowStr(0, 0, (uint8_t* )buf, 1);
		sprintf(buf, "anx=%.2f any=%.2f  ", Flag.Anglex, Flag.Angley);
		OLED_ShowStr(0, 1, (uint8_t* )buf, 1);
		sprintf(buf, "acx = %03d acy = %03d  ", Flag.x_actual, Flag.y_actual);
		OLED_ShowStr(0, 2, (uint8_t* )buf, 1);
		sprintf(buf, "  %d     ", Flag.Is_Mode_Set);
		OLED_ShowStr(0, 7, (uint8_t* )buf, 1);
	}
	else if(Flag.Is_OLED_Face == 3) {
		sprintf(buf, "dx = %03d  dy = %03d  ", Flag.dx, Flag.dy);
		OLED_ShowStr(0, 0, (uint8_t* )buf, 1);
		sprintf(buf, "acx = %03d acy = %03d  ", Flag.x_actual, Flag.y_actual);
		OLED_ShowStr(0, 1, (uint8_t* )buf, 1);
		sprintf(buf, "slope= %0.3f    ", Flag.Slope);
		OLED_ShowStr(0, 2, (uint8_t* )buf, 1);
		sprintf(buf, "i = %d      ", i);
		OLED_ShowStr(0, 3, (uint8_t* )buf, 1);
		sprintf(buf, "angley = %.2f      ", angley);
		OLED_ShowStr(0, 4, (uint8_t* )buf, 1);
		sprintf(buf, "anglex = %.2f  ", anglex);
		OLED_ShowStr(0, 5, (uint8_t* )buf, 1);
		sprintf(buf, " %d  %d  ", Flag.dx, Flag.dy);
		OLED_ShowStr(0, 6, (uint8_t* )buf, 1);
		sprintf(buf, "  %d   %d  ", Black_Retanx[0], Black_Retany[0]);
		OLED_ShowStr(0, 7, (uint8_t* )buf, 1);		
	}
	else if(Flag.Is_OLED_Face == 4) {
		sprintf(buf, "x1 = %03d  y1 = %03d  ", RetangleX[0], RetangleY[0]);
		OLED_ShowStr(0, 0, (uint8_t* )buf, 1);
		sprintf(buf, "x2 = %03d  y2 = %03d  ", RetangleX[1], RetangleY[1]);
		OLED_ShowStr(0, 1, (uint8_t* )buf, 1);
		sprintf(buf, "x3 = %03d  y3 = %03d  ", RetangleX[2], RetangleY[2]);
		OLED_ShowStr(0, 2, (uint8_t* )buf, 1);
		sprintf(buf, "Test = %d      ", Flag.TEST);
		OLED_ShowStr(0, 3, (uint8_t* )buf, 1);
		sprintf(buf, "x4 = %03d  y4 = %03d  ", RetangleX[3], RetangleY[3]);
		OLED_ShowStr(0, 4, (uint8_t* )buf, 1);
		sprintf(buf, "x5 = %03d  y5 = %03d  ", Flag.x_centry, Flag.y_centry);
		OLED_ShowStr(0, 5, (uint8_t* )buf, 1);
		sprintf(buf, " ");
		OLED_ShowStr(0, 6, (uint8_t* )buf, 1);
		sprintf(buf, "  ");
		OLED_ShowStr(0, 7, (uint8_t* )buf, 1);		
	}
	else if(Flag.Is_OLED_Face == 5) {
		sprintf(buf, "x1 = %03d  y1 = %03d  ", Black_Retanx[0], Black_Retany[0]);
		OLED_ShowStr(0, 0, (uint8_t* )buf, 1);
		sprintf(buf, "x2 = %03d  y2 = %03d  ", Black_Retanx[1], Black_Retany[1]);
		OLED_ShowStr(0, 1, (uint8_t* )buf, 1);
		sprintf(buf, "x3 = %03d  y3 = %03d  ", Black_Retanx[2], Black_Retany[2]);
		OLED_ShowStr(0, 2, (uint8_t* )buf, 1);
		sprintf(buf, "Test = %d      ", Flag.TEST);
		OLED_ShowStr(0, 3, (uint8_t* )buf, 1);
		sprintf(buf, "x4 = %03d  y4 = %03d  ", Black_Retanx[3], Black_Retany[3]);
		OLED_ShowStr(0, 4, (uint8_t* )buf, 1);
		sprintf(buf, "x5 = %03d  y5 = %03d  ", Flag.Is_Uart_Rec, Flag.Is_Mode_Set);
		OLED_ShowStr(0, 5, (uint8_t* )buf, 1);
		sprintf(buf, "%d  ", Flag.Cnt);
		OLED_ShowStr(0, 6, (uint8_t* )buf, 1);
		sprintf(buf, " mode = %d ", Flag.Is_Mode_Set);
		OLED_ShowStr(0, 7, (uint8_t* )buf, 1);		
	}
}

void VOFA_Proc() {
	
	static __IO uint32_t uwTick_VOFA_Speed;
	
	if(uwTick - uwTick_VOFA_Speed < 50) return;
	uwTick_VOFA_Speed = uwTick;
	
//	print(&huart1, "%d, %d, %d, %d\n" ,Flag.X_AXIS, Flag.Y_AXIS, Flag.x_actual, Flag.y_actual);
	print(&huart1, "%.2f, %.2f, %.2f, %.2f\n" ,anglex, angley, Flag.Anglex, Flag.Angley);
	
}

void PARAM_Init(void) {
	Flag.Is_Angle_Set = 1;
	Flag.Is_OLED_Face = 3;
	Flag.Is_Mode_Set = 0;
	Flag.Anglex = 0;
	Flag.Angley = 0;   // 设置舵机的初始复位角度 PID闭环控制
	Flag.x_centry = 120;
	Flag.y_centry = 98;
	Flag.FSTATE = Centy_To_Start;
	
}

// 计算一次函数的斜率
float calculateSlope(float x1, float y1, float x2, float y2) {
    return ((y2 - y1) / (x2 - x1));
}

// 计算一次函数的截距 其中slope是斜率
int calculateIntercept(int x1, int y1, float slope) {
    return (y1 - slope * x1);
}

// 根据一次函数的斜率和截距，计算 y 值  intercept是截距   传入x的值计算y值
int calculateY(int x, float slope, int intercept) {
    return slope * x + intercept;
}

// 根据一次函数的斜率和截距，计算 x 值  传入y计算x的值
int calculateX(float y, float slope, float intercept) {
    return (y - intercept) / slope;
}

int myabs(int p) {
	int q;
	q = p > 0? p : (-p);
	return q;
}
/******************************************END_OF_SHOW**************************************************/

/*******************************************控制层函数*******************************************/
// 运动目标控制处理函数
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
					if(myabs(Flag.y_actual - RetangleY[0]) < 1) {
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


/*******************************************end of file*******************************************/