#include "control.h"
#include "stdarg.h"
#include "string.h"
#include "dma.h"
#include "math.h"

PARAM_t Flag;

float WheelOneCircleDis=WheelDiameter*pi;//定义小车轮子行走一圈距离变量

int Rpm_Encoder_Pulse_MAX;

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	
	if(htim->Instance == TIM4){
		
		//1.单位时间脉冲数
		Motor1.Uinttime_MotorPulse = GetMotorPulse(1);
		Motor2.Uinttime_MotorPulse = GetMotorPulse(3);

	  //将读取到的单位时间内的脉冲数转为单位为rpm
		Motor1.speed = (float)(Motor1.Uinttime_MotorPulse/(52.0*30.0*10.0))*1000.0*60.0;
		Motor2.speed = (float)(Motor2.Uinttime_MotorPulse/(52.0*30.0*10.0))*1000.0*60.0;
		//2.累计脉冲数
		Motor1.Sigma_MotorPulse += Motor1.Uinttime_MotorPulse;
		Motor2.Sigma_MotorPulse += Motor2.Uinttime_MotorPulse;
		
		
		if(Flag.Is_Location_Speed == 1) {
			
			//3.位置环
			Motor1.Target_Position = Nlaps_Encoder_Cnt(Motor1.Num);
			Motor2.Target_Position = Nlaps_Encoder_Cnt(Motor2.Num);
			
			Location.Motor1_Out = Location_PID_Realize(Motor1.Sigma_MotorPulse, Motor1.Target_Position);
			Location.Motor2_Out = Location_PID_Realize(Motor2.Sigma_MotorPulse, Motor2.Target_Position);
			
			//限幅值220；
			//位置环限幅
			if(Location.Motor1_Out>MOTOR_SPEED_MAX)Location.Motor1_Out = MOTOR_SPEED_MAX;
			else if(Location.Motor1_Out<-MOTOR_SPEED_MAX)Location.Motor1_Out = -MOTOR_SPEED_MAX;
			
			if(Location.Motor2_Out>MOTOR_SPEED_MAX)Location.Motor2_Out = MOTOR_SPEED_MAX;
			else if(Location.Motor2_Out<-MOTOR_SPEED_MAX)Location.Motor2_Out = -MOTOR_SPEED_MAX;

			// 速度环  此处定义两个速度环函数，由于上一次传参值会影响下一次的新的传参值，为避免影响，因此定义两个速度环函数
			Velocity.Motor1_Out = Velocity_PID_Realize(Motor1.speed, Location.Motor1_Out);
			Velocity.Motor2_Out = Velocity_PID_Realize1(Motor2.speed, Location.Motor2_Out);
		
			LIMIT(Velocity.Motor1_Out, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
			LIMIT(Velocity.Motor2_Out, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
			
			if(gfp_abs(Velocity.Motor1_Out) <= 300)Velocity.Motor1_Out = 0;
			if(gfp_abs(Velocity.Motor2_Out) <= 300)Velocity.Motor2_Out = 0;
			
			Set_PWM(Velocity.Motor1_Out, Velocity.Motor2_Out);
		}
	}
}
	
void print(UART_HandleTypeDef* huart, const char* buf, ...) {
  char str[255] = {0};
  va_list v;
  va_start(v, buf);
  vsprintf(str, buf, v); //使用可变参数的字符串打印。类似sprintf
//	HAL_UART_Transmit(huart, (uint8_t* )str, strlen(str), 1000); 
	if(HAL_UART_Transmit_DMA(huart, (uint8_t* )str, strlen(str)) != HAL_OK) {
		Error_Handler(); 
	}		// DMA模式发送
	
  va_end(v);
}

void Vofa_Proc(void) {
	static __IO uint32_t uwTick_VOFA_Speed;
	if(uwTick - uwTick_VOFA_Speed < 100) return;
	uwTick_VOFA_Speed = uwTick;
	
//	printf("%d\n", Motor1.Uinttime_MotorPulse);
	print(&huart1, "%d, %.2f\n", Location.Motor1_Out, Motor1.speed);
//	print(&huart1, "%d\n", Motor2.Uinttime_MotorPulse);
}

/*****************************************小车循迹底层函数************************************************************/

/**
 *@brief:根据循迹传感器pid调节小车转向使小车处于黑线中间
 * @param:
 *        [in]int TraceDate: 循迹传感器输出的值
 * @return: 返回调节电机速度的转向pwm
 */
int ChangeTraceTurn(int TraceDate)
{
	int pwm=0;
	int bias;
	bias = TraceDate;
	pwm = Turn_PID_Realize( bias);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//限幅
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:根据pid调节左边电机到目标速度
 * @param:
 *        [in]int EncodeSpdL: 当前左电机编码器测速值
 *        [in]float TarSpdL:左边电机目标速度,最大速度越1.19m/s
 * @return: 返回左边电机计算后的pwm占空比
 */
int ChangeSpeedMotorL(int NowEncodeSpdL,float TarSpdL)
{
	int pwm=0;
	int TarEncodeSpdL;
	TarEncodeSpdL=(int)((TarSpdL*ACircleEncoder)/(WheelOneCircleDis*100));//根据目标速度求出目标编码器速度

	pwm=Position_PID_Realize(NowEncodeSpdL, TarEncodeSpdL);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//限幅
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:根据pid调节右边电机到目标速度
 * @param:
 *        [in]int NowEncodeSpdR: 当前右电机编码器测速值
 *        [in]float TarSpdR:右边电机目标速度,根据限幅最大速度约为0.7m/s
 * @return: 返回右边电机计算后的pwm占空比
 */
int ChangeSpeedMotorR(int NowEncodeSpdR,float TarSpdR)
{
	int pwm=0;
	int TarEncodeSpdR;
	TarEncodeSpdR=(int)((TarSpdR*ACircleEncoder)/(WheelOneCircleDis*100));//根据目标速度求出目标编码器速度
	
	pwm=Position_PID_Realize(NowEncodeSpdR, TarEncodeSpdR);
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//限幅
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	return pwm;
}

/**
 *@brief: 让小车根据循迹黑线走
 *@param:
 *        [in]TraceDate: 循迹传感器输出的值
 *        [in]TarSpeed:循迹的目标速度,速度大小0-0.7m/s
 *@return: 到达目标点返回1，否则返回0
 */
void TraceMove(int TraceDate,float TarSpeed)
{
	int turnpwm=0;
	int spdpwml=0,spdpwmr=0;
	int pwml=0,pwmr=0;
	
	turnpwm=ChangeTraceTurn(TraceDate);//转向PID
	
	spdpwml=ChangeSpeedMotorL(Motor2.Uinttime_MotorPulse, TarSpeed);
	spdpwmr=ChangeSpeedMotorR(Motor1.Uinttime_MotorPulse, TarSpeed);

//	printf("Encode_Left:%d Encode_Right:%d\r\n",Encode_Left,Encode_Right);
	
	pwmr=turnpwm+spdpwmr;
	if(pwmr>MAX_MOTOR_PWM)		pwmr=MAX_MOTOR_PWM;//限幅
	else if(pwmr<-MAX_MOTOR_PWM)  pwmr=-MAX_MOTOR_PWM;
	
	pwml=-turnpwm+spdpwml;
	if(pwml>MAX_MOTOR_PWM)		pwml=MAX_MOTOR_PWM;//限幅
	else if(pwml<-MAX_MOTOR_PWM)  pwml=-MAX_MOTOR_PWM;
	
	Set_PWM(pwml, pwmr);
}

/*@brief:根据小车的目标距离pid调节左边电机速度
 * @param:
 *        [in]int NowEncodeLALL: 当前左电机编码器总里程
 *        [in]float TarDis:左边电机目标距离，0-10M
 * @return: 返回左边电机计算后的pwm占空比
 */
int ChangeDisSpeedMotorL(int NowEncodeLALL,float TarDis)
{
	int pwm=0;
	int TarEncodeLALL;
	TarEncodeLALL=(int)(((TarDis/WheelOneCircleDis)*ACircleEncoder));//根据目标距离求出目标编码器值
	pwm = Position_PID_Realize1(NowEncodeLALL, TarEncodeLALL);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//限幅
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:根据小车的目标距离pid调节右边电机速度
 * @param:
 *        [in]int NowEncodeRALL: 当前右电机编码器总里程
 *        [in]float TarDis:左边电机目标距离，0-10M
 * @return: 返回右边电机计算后的pwm占空比
 */
int ChangeDisSpeedMotorR(int NowEncodeRALL,float TarDis)
{
	int pwm=0;
	int TarEncodeRALL;
	TarEncodeRALL=(int)(((TarDis/WheelOneCircleDis)*ACircleEncoder));//根据目标距离求出目标编码器值
	pwm = Position_PID_Realize1(NowEncodeRALL, TarEncodeRALL);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//限幅
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief: 根据目标距离循迹行驶，当行驶目标距离后停止循迹
 *@param:
 *        [in]TraceDate: 循迹传感器输出的值
 *        [in]TarDis:小车行驶的目标距离,输出范围暂定
 *@return: 到达目标点返回1，否则返回0
 */
void TraceMoveWithDis(int TraceDate,float TarDis)
{
	int turnpwm=0;
	int spdlpwm,spdrpwm;
	int pwml=0,pwmr=0;
	int TarEncodeRALL=(int)(((TarDis/WheelOneCircleDis)*ACircleEncoder)); // 根据目标距离求出目标编码器值
	
	turnpwm=ChangeTraceTurn(TraceDate);
	
	spdlpwm = ChangeDisSpeedMotorL(Motor1.Sigma_MotorPulse, TarDis);
	spdrpwm = ChangeDisSpeedMotorR(Motor2.Sigma_MotorPulse, TarDis);
		
	pwmr=turnpwm+spdrpwm;
	
	if(pwmr>MAX_MOTOR_PWM)		pwmr=MAX_MOTOR_PWM;//限幅
	else if(pwmr<-MAX_MOTOR_PWM)  pwmr=-MAX_MOTOR_PWM;
	pwml=-turnpwm+spdlpwm;
	if(pwml>MAX_MOTOR_PWM)		pwml=MAX_MOTOR_PWM;//限幅
	else if(pwml<-MAX_MOTOR_PWM)  pwml=-MAX_MOTOR_PWM;
	
	Set_PWM(pwml, pwmr);
	
	if((gfp_abs(TarEncodeRALL - ((int)(Motor1.Sigma_MotorPulse + Motor2.Sigma_MotorPulse)/2)) < 1000)) {
			
			Motor_Stop();
			Flag.Is_DisArrive = 1;  // 停车标志位
		}
}

/*@brief:通过PD控制器控制小车行走一定的距离
 * @param:
 *        [in]float NowEncodeLALL: 当前左轮编码器总里程
 *        [in]float NowEncodeRALL: 当前右轮编码器总里程
 *        [in]float TarDisL: 左边电机目标距离，0-10M
 *        [in]float TarDisR:右边电机目标距离，0-10M
 *@return: 行走结束返回0，否者返回1
 */
void MoveDis(int NowEncodeLALL,int NowEncodeRALL,float TarDisL,float TarDisR)
{
	int dislpwm,disrpwm;
	
	dislpwm=ChangeDisSpeedMotorL(NowEncodeLALL,TarDisL);
	disrpwm=ChangeDisSpeedMotorR(NowEncodeRALL,TarDisR);
	
	if(gfp_abs(dislpwm)<300) dislpwm=0;//防止电机转不动空耗电
	if(gfp_abs(disrpwm)<300) disrpwm=0;
	
	Set_PWM(dislpwm, disrpwm);
}

/************************************End of File***********************************************/

/**********************************小车控制应用层***********************************************/
void OLED_Proc(void) {
	
	static __IO uint32_t uwTick_OLED_Speed;
	char buf[22];
	
	if(uwTick - uwTick_OLED_Speed < 100) return;
	uwTick_OLED_Speed = uwTick;
	
	if(Flag.OLED_FACE == 0) {
		sprintf(buf, "0 = %d ", Motor1.Uinttime_MotorPulse);
		OLED_ShowStr(0, 0, (unsigned char* )buf, 1);
		sprintf(buf, "1 = %d ", Motor2.Uinttime_MotorPulse);
		OLED_ShowStr(0, 1, (unsigned char* )buf, 1);
		
		sprintf(buf, "2 = %d    %.2f ", Location.Motor1_Out, Motor1.Target_Position);
		OLED_ShowStr(0, 2, (unsigned char* )buf, 1);
		sprintf(buf, "3 = %d    %.2f ", Location.Motor2_Out, Motor2.Target_Position);
		OLED_ShowStr(0, 3, (unsigned char* )buf, 1);
		
		
		sprintf(buf, "M1 =  %.2f    %d    ", Motor1.speed, Velocity.Motor1_Out);
		OLED_ShowStr(0, 4, (unsigned char* )buf, 1);
		sprintf(buf, "M2 = %.2f     %d    ", Motor2.speed, Velocity.Motor2_Out);
		OLED_ShowStr(0, 5, (unsigned char* )buf, 1);
		
		sprintf(buf, "6 = %d ", Rpm_Encoder_Pulse_MAX);
		OLED_ShowStr(0, 6, (unsigned char* )buf, 1);
	}
	else if(Flag.OLED_FACE == 1) {
		
		sprintf(buf, "0 = %d ", (int)(((0.3/WheelOneCircleDis)*ACircleEncoder)));
		OLED_ShowStr(0, 0, (unsigned char* )buf, 1);
		
		sprintf(buf, "1 = %d ", (int)(Motor1.Sigma_MotorPulse + Motor2.Sigma_MotorPulse)/2);
		OLED_ShowStr(0, 1, (unsigned char* )buf, 1);
		
		sprintf(buf, "S_R:%d R_T:%d  ", Flag.Is_Select_Right, Flag.Is_Right_Turn);
		OLED_ShowStr(0, 2, (unsigned char* )buf, 1);
		
		sprintf(buf, "Is_DisArrive = %d ", Flag.Is_DisArrive);
		OLED_ShowStr(0, 3, (unsigned char* )buf, 1);
		
		sprintf(buf, "Is_Track = %d ", Flag.Is_Track);
		OLED_ShowStr(0, 4, (unsigned char* )buf, 1);
		
		sprintf(buf, "Is_Car_Stop = %d ", Flag.Is_Car_Stop);
		OLED_ShowStr(0, 5, (unsigned char* )buf, 1);
		
		sprintf(buf, "NumOne = %d ", Flag.Is_NumOne);
		OLED_ShowStr(0, 6, (unsigned char* )buf, 1);
		sprintf(buf, "NumTwo = %d ", Flag.Is_NumTwo);
		OLED_ShowStr(0, 7, (unsigned char* )buf, 1);
		
	}
	else if(Flag.OLED_FACE == 2) {
		sprintf(buf, "0 = %d ", Get_Adc(0));
		OLED_ShowStr(0, 0, (unsigned char* )buf, 1);
		
		sprintf(buf, "1 = %d ", Get_Adc(8));
		OLED_ShowStr(0, 1, (unsigned char* )buf, 1);
		
		sprintf(buf, "2 =  %d ", Get_Adc(4));
		OLED_ShowStr(0, 2, (unsigned char* )buf, 1);
		
		sprintf(buf, "Is_DisArrive = %d ", Flag.Is_DisArrive);
		OLED_ShowStr(0, 3, (unsigned char* )buf, 1);
		
		sprintf(buf, " = ");
		OLED_ShowStr(0, 4, (unsigned char* )buf, 1);
		
		sprintf(buf, "5 = ");
		OLED_ShowStr(0, 5, (unsigned char* )buf, 1);
	}
	else if(Flag.OLED_FACE == 3) {
		sprintf(buf, "Sure_NumFour[1] = %d ", Flag.Is_Sure_NumFour);
		OLED_ShowStr(0, 0, (unsigned char* )buf, 1);
		
		sprintf(buf, "four =  %d ", Flag.Is_NumFour);
		OLED_ShowStr(0, 1, (unsigned char* )buf, 1);
		
		sprintf(buf, "Sure_NumFive =  %d ", Flag.Is_Sure_NumFive);
		OLED_ShowStr(0, 2, (unsigned char* )buf, 1);
		
		sprintf(buf, "six =   %d ", Flag.Is_NumSix);
		OLED_ShowStr(0, 3, (unsigned char* )buf, 1);
		
		sprintf(buf, "seven = %d ", Flag.Is_NumSeven);
		OLED_ShowStr(0, 4, (unsigned char* )buf, 1);
		
		sprintf(buf, "eight = %d ", Flag.Is_DIs_MID_FAR);
		OLED_ShowStr(0, 5, (unsigned char* )buf, 1);
	}
	else if(Flag.OLED_FACE == 4) {
		sprintf(buf, "Sure_NumThree = %d %d ", Flag.Is_Sure_NumThree, Flag.Is_NumThree);
		OLED_ShowStr(0, 0, (unsigned char* )buf, 1);
		
		sprintf(buf, "Sure_NumFour =  %d %d ", Flag.Is_Sure_NumFour, Flag.Is_NumFour);
		OLED_ShowStr(0, 1, (unsigned char* )buf, 1);
		
		sprintf(buf, "Sure_NumFive =  %d %d ", Flag.Is_Sure_NumFive, Flag.Is_NumFive);
		OLED_ShowStr(0, 2, (unsigned char* )buf, 1);
		
		sprintf(buf, "Sure_NumSix =   %d %d ", Flag.Is_Sure_NumSix, Flag.Is_NumSix);
		OLED_ShowStr(0, 3, (unsigned char* )buf, 1);
		
		sprintf(buf, "Sure_NumSeven = %d %d ", Flag.Is_Sure_NumSeven, Flag.Is_NumSeven);
		OLED_ShowStr(0, 4, (unsigned char* )buf, 1);
		
		sprintf(buf, "eight = %d ", Flag.Is_DIs_MID_FAR);
		OLED_ShowStr(0, 5, (unsigned char* )buf, 1);
	}
}

void Param_Init(void) {

	Flag.Is_Location_Speed = 0;   // 串级速度位置PID是否开启标志位
	Flag.Is_Track = 0;
	Flag.OLED_FACE = 4;           // OLED显示界面
	Flag.Is_Medicine = 0;         // 开始无药品
	Flag.Is_DisArrive = 0;        // 0表示开始距离未到达
	Flag.Is_Select_Left = 0;
	Flag.Is_Select_Right = 0;
	Flag.Is_Left_Log = 0;
	Flag.Is_Left_Turn = 0;
	Flag.Is_Right_Log = 0;
	Flag.Is_Right_Turn = 0;
	Flag.Is_NumOne = 0;
	Flag.Is_NumTwo = 0;
	Flag.Is_Num_Recongnize = 1;
}

#define Follow_Dis 0.85      // 近端直行距离
#define Turn_Dis   0.48      // 返回距离
#define Follow_MID_Dis 1.445  // 中端直行距离   距离到了进行识别
#define Follow_FAR_Dis 2.65  // 远端直行距离  再议
#define Follow_Keep_Dis 0.38  // 

extern int TraceDate;
CAR_t CAR;

void Car_State_Machine(void) {
	switch(CAR.Car_State) {
		case Test_Medicine_State:          // 检测药品状态
				if(Flag.Is_Medicine == 1) {    // 如果检测到了药品
					Flag.Is_Track = 1;
					if(Flag.Is_NumOne == 1) {    // 如果识别到了数字1
						Flag.Is_NumOne = 0;
						Flag.Is_Select_Left = 1;   // 开启左转弯标志
						CAR.Car_Dis = Follow_Dis;  // 设置直行距离
						CAR.Car_State = Follow_Line_State;  // 进入直行状态
					}
					else if(Flag.Is_NumTwo == 1) {  // 如果识别到了数字2
						Flag.Is_NumTwo = 0;
						Flag.Is_Select_Right = 1;
						CAR.Car_Dis = Follow_Dis;
						CAR.Car_State = Follow_Line_State;
					}
					// 中远端数字
					else if(Flag.Is_DIs_MID_FAR == 1) {   // 如果识别到了中远端数字
						CAR.Car_Dis = Follow_MID_Dis;
						Flag.Is_Mid_Dis = 1;    // 识别到中远端数字，将中端返回距离标志置为1
						
						CAR.Car_State = Follow_Line_State;
						if(Flag.Is_NumThree == 1) {  // 如果识别到了数字3
							Flag.Is_NumThree = 0;
							Flag.Is_Sure_NumThree = 1;  // 保留本次识别，若左侧再次识别，将本标志置为1，若右侧再次识别，将本标志位置为2
						}
						else if(Flag.Is_NumFour == 1) {  // 如果识别到了数字4
							Flag.Is_NumFour = 0;
							Flag.Is_Sure_NumFour=1;
						}
						else if(Flag.Is_NumFive == 1) {
							Flag.Is_NumFive = 0;
							Flag.Is_Sure_NumFive =1;
						}
						else if(Flag.Is_NumSix == 1) {
							Flag.Is_NumSix = 0;
							Flag.Is_Sure_NumSix =1;
						}
						else if(Flag.Is_NumSeven == 1) {
							Flag.Is_NumSeven = 0;
							Flag.Is_Sure_NumSeven =1;
						}
//						Flag.Is_Num_Recongnize = 0;  // 关闭数字识别   开始可以不用关闭数字识别
					}
				}

			break;
		case Follow_Line_State:     // 前进状态
			if(Flag.Is_Track == 1) {
				if(TraceDate == 30000) {
					Motor_Stop();
				}
				else {
					TraceMoveWithDis(TraceDate, CAR.Car_Dis);
					if(Flag.Is_DisArrive == 1) {  // 距离到了
						CAR.Car_State = Cross_State;
						Motor1.Sigma_MotorPulse = 0;  // 清零避免对后续转弯产生影响
						Motor2.Sigma_MotorPulse = 0;
						Flag.Is_Track = 0;
						Flag.Is_NumOne = 0;  // 将数字1的标志位清零
						Flag.Is_NumTwo = 0;  // 清零数字2标志位
						
						if(Flag.Is_Medicine == 1) {  //如果药品在车上，且到了送药病房，点亮红灯
							if(fabs(CAR.Car_Dis - Turn_Dis) < 0.1) 
								Red_Set;           // 点亮红色小灯
						}
					}
				}
			}
			
			break;
		case Cross_State:       // 等待状态
			Flag.Is_DisArrive = 0;
			if(Flag.Is_Select_Left == 1) {
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Select_Left = 0;      // 清零左转弯标志位
					// 左转指令
				CAR.Car_State = Select_Left_State;  // 进入左转弯状态
				Flag.Is_Location_Speed = 1;         // 开启位置速度环PID
			}
			else if(Flag.Is_Select_Right == 1) {
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Select_Right = 0;     // 清零右转弯标志位
					// 右转指令
				CAR.Car_State = Select_Right_State;  // 进入右转弯状态
				Flag.Is_Location_Speed = 1;
			}
			else if(Flag.Is_Medicine == 0) {   // 药品被取下
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Location_Speed = 1;      // 开启位置速度闭环控制
				
				if(Flag.Is_Right_Turn)           // 开始进行左转弯，再回来的路上进行了右转弯，则进入停车状态
					CAR.Car_State = Stop_State;
				else if(Flag.Is_Left_Turn)
					CAR.Car_State = Stop_State;
				else 
					CAR.Car_State = Turn_Around_State;
			}
			else if(Flag.Is_DIs_MID_FAR == 1) {     // 如果直行到了，中远端第一个识别路口
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Location_Speed = 1;
				CAR.Car_State = Select_MID_Left_State;
				Flag.Is_Num_Recongnize = 1;   // 开启数字识别
			}
			else if(Flag.Is_Car_Stop == 1) {
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Location_Speed = 0;
				CAR.Car_State = Stop_State;
			}
			break;
		case Select_Left_State:  // 左转弯状态
			if(Flag.Is_Location_Speed == 1) {
				Motor1.Num = Car_Turn(-90);
				Motor2.Num = Car_Turn(90);  
				HAL_Delay(500);
				Flag.Is_Left_Log = 1;   // 左转弯完成后，记录一次
//				if(gfp_abs(Get_Adc(0) - Get_Adc(4)) < 550) {
					Flag.Is_Location_Speed = 0;      // 关闭速度位置环PID
					Flag.Is_Track = 1;               // 开启循迹标志位
					CAR.Car_State = Follow_Line_State;
					Motor1.Sigma_MotorPulse = 0;
					Motor2.Sigma_MotorPulse = 0;
					Motor1.Num = Motor2.Num = 0;
					
					if(Flag.Is_Left_Turn == 1) {
						if(Flag.Is_Mid_Dis) {  // 如果是中端返回
							Flag.Is_Mid_Dis  = 0;
							CAR.Car_Dis = Follow_MID_Dis + Follow_Keep_Dis - 0.21;
						}
						else
							CAR.Car_Dis = Follow_Dis;
						
						Flag.Is_Car_Stop = 1;
					}
					else {
						CAR.Car_Dis = Turn_Dis;   // 近中端，转向180°后，行驶0.48m
					}
//				}
			}

			break;
		case Select_Right_State:        // 右转弯状态
			if(Flag.Is_Location_Speed == 1) {
				Motor1.Num = Car_Turn(90);
				Motor2.Num = Car_Turn(-90);
				HAL_Delay(500);
				Flag.Is_Right_Log = 1;   // 右转弯完成后，记录一次
				
				Flag.Is_Location_Speed = 0;      // 关闭速度位置环PID
				Flag.Is_Track = 1;               // 开启循迹标志位
				CAR.Car_State = Follow_Line_State;
				Motor1.Sigma_MotorPulse = 0;
				Motor2.Sigma_MotorPulse = 0;
				Motor1.Num = Motor2.Num = 0;
				
				if(Flag.Is_Right_Turn == 1) {    // 在返回路程进行右转弯
					
					if(Flag.Is_Mid_Dis) {  // 已解决   如果是中端
						Flag.Is_Mid_Dis = 0;
						CAR.Car_Dis = Follow_MID_Dis + Follow_Keep_Dis - 0.21;  // 设置中端返回距离
					}
					else
						CAR.Car_Dis = Follow_Dis;    // 否则设置近端返回距离
					
//					CAR.Car_Dis = Follow_Dis;      // 设置右转后，返回路程的直行距离
					Flag.Is_Car_Stop = 1;          // 这里暂时没有用到这个标志， 将停车标志置为1
				}
				else {
					CAR.Car_Dis = Turn_Dis;        // 否则设置正常路程的右转弯距离
				}
			}
			break;
		case Turn_Around_State:        // 掉头转向状态
			Motor1.Num = Car_Turn(180);
			Motor2.Num = Car_Turn(-180);
			HAL_Delay(700);
			if(gfp_abs(Get_Adc(8) - Get_Adc(4)) < 550) {
				Flag.Is_Location_Speed = 0;
				Flag.Is_Track = 1;
				CAR.Car_State = Follow_Line_State;     // 返回直行状态
				CAR.Car_Dis = Turn_Dis;
				Motor1.Sigma_MotorPulse = 0;
				Motor2.Sigma_MotorPulse = 0;
				Motor1.Num = Motor2.Num = 0;
				
				if(Flag.Is_Left_Log == 1) {   // 如果发生了一次左转弯，则回去的路上将进行右转弯
					Flag.Is_Left_Log = 0;       // 将记录左转弯标志清零
					Flag.Is_Select_Right = 1;   // 开启右转弯标志
					Flag.Is_Right_Turn = 1;     // 将返回路程的右转弯标志置为1
				}
				else if(Flag.Is_Right_Log == 1) {  // 如果发生了一次右转弯，则回去的路上将进行左转弯
					Flag.Is_Right_Log = 0;
					Flag.Is_Select_Left = 1;
					Flag.Is_Left_Turn = 1;
				}		
			}
			break;
		case Stop_State:    // 停车状态
			Flag.Is_Location_Speed = 0;     // 关闭速度位置环
			Flag.Is_Track = 0;              // 关闭循迹功能
			Motor_Stop();                   // 进行停车
			Green_Set;      // 回到起点，开启绿灯
			break;
		case Select_MID_Left_State:      // 先进行左转识别
			Flag.Is_Num_Recongnize = 1;
			Motor1.Num = Car_Turn(-15);
			Motor2.Num = Car_Turn(15);
			HAL_Delay(2000);
			// 如果再次识别到了目标
			if(Flag.Is_Sure_NumThree == 1)
				while(Flag.Is_NumThree == 1) {
					
					Flag.Is_Sure_NumThree = 2;   // 左转识别，将再次识别的目标置为2
					if(2 == Flag.Is_Sure_NumThree) {
						Flag.Is_NumThree = 0;
					}
				}
			else if(Flag.Is_Sure_NumFour == 1)
				while(Flag.Is_NumFour == 1) {
					Flag.Is_Sure_NumFour = 2;
					if(2 == Flag.Is_Sure_NumFour) {
						Flag.Is_NumFour = 0;	
					}
				}
			else if(Flag.Is_Sure_NumFive == 1)
				while(Flag.Is_NumFive == 1) {
					
					Flag.Is_Sure_NumFive = 2;
					if(Flag.Is_Sure_NumFive == 2) {
						Flag.Is_NumFive = 0;	
					}
				}
			else if(Flag.Is_Sure_NumSix == 1)
				while(Flag.Is_NumSix == 1) {
					
					Flag.Is_Sure_NumSix = 2;
					if(Flag.Is_Sure_NumSix == 2) {
						Flag.Is_NumSix = 0;
					}
				}
			else if(Flag.Is_Sure_NumSeven == 1)
				while(Flag.Is_NumSeven == 1) {
					
					Flag.Is_Sure_NumSeven = 2;
					if(Flag.Is_Sure_NumSix == 2) {
						Flag.Is_NumSeven = 0;
					}
				}
				
			
			CAR.Car_State = Select_MID_Right_State;
			Motor1.Sigma_MotorPulse = 0;
			Motor2.Sigma_MotorPulse = 0;
			Motor1.Num = Motor2.Num = 0;

			break;
		case Select_MID_Right_State:     // 在进行右转识别
			Flag.Is_Num_Recongnize = 1; 
			Motor1.Num = Car_Turn(35);
			Motor2.Num = Car_Turn(-35);
			HAL_Delay(2000);   // 延时两秒是由于串口回传数据比较慢
			// 如果再次识别到了目标
			if(Flag.Is_Sure_NumThree == 1)
				while(Flag.Is_NumThree == 1) {
					
					Flag.Is_Sure_NumThree = 3;    // 右转识别， 将再次识别的目标置为3
					if(Flag.Is_Sure_NumThree == 3) {
						Flag.Is_NumThree = 0;
					}
				}
			else if(Flag.Is_Sure_NumFour == 1)
				while(Flag.Is_NumFour == 1) {
					
					Flag.Is_Sure_NumFour = 3;
					if(Flag.Is_Sure_NumFour == 3) {
						Flag.Is_NumFour = 0;
					}
				}
			else if(Flag.Is_Sure_NumFive == 1)
				while(Flag.Is_NumFive == 1) {
					
					Flag.Is_Sure_NumFive = 3;
					if(Flag.Is_Sure_NumFive == 3) {
						Flag.Is_NumFive = 0;
					}
				}
			else if(Flag.Is_Sure_NumSix == 1)
				while(Flag.Is_NumSix == 1) {
					
					Flag.Is_Sure_NumSix = 3;
					if(Flag.Is_Sure_NumSix == 3) {
						Flag.Is_NumSix = 0;
					}
				}
			else if(Flag.Is_Sure_NumSeven == 1)
				while(Flag.Is_NumSeven == 1) {
					
					Flag.Is_Sure_NumSeven = 3;
					if(Flag.Is_Sure_NumSeven == 3) {
						Flag.Is_NumSeven = 0;
					}
				}		
			CAR.Car_State = Turn_Follow_State;  // 进入车头摆正状态
			Motor1.Sigma_MotorPulse = 0;
			Motor2.Sigma_MotorPulse = 0;
			Motor1.Num = Motor2.Num = 0;
			
			break;
		case Turn_Follow_State:    // 将车头转回来
			Motor1.Num = Car_Turn(-20);
			Motor2.Num = Car_Turn(20);
			HAL_Delay(700);
			Motor1.Sigma_MotorPulse = 0;
			Motor2.Sigma_MotorPulse = 0;
			Motor1.Num = Motor2.Num = 0;
			Flag.Is_Num_Recongnize = 0;     // 关闭数字识别
			Flag.Is_Location_Speed = 0;  // 关闭速度位置环
			Flag.Is_Track = 1;           // 开启循迹
			CAR.Car_State = Keep_Fllow_Straight_State;  // 进入继续保持执行的状态
			CAR.Car_Dis = Follow_Keep_Dis;              // 设置继续行驶的距离
			break;
		case Keep_Fllow_Straight_State:  // 车头转向回来过后，保持继续直行的状态
			
			if(Flag.Is_Track == 1) {
				if(TraceDate == 30000) {
					Motor_Stop();
				}
				else {
					TraceMoveWithDis(TraceDate, CAR.Car_Dis);
					Flag.Is_DIs_MID_FAR = 0;
					if(Flag.Is_DisArrive == 1) {  // 距离到了
						CAR.Car_State = Cross_State;
						Motor1.Sigma_MotorPulse = 0;  // 清零避免对后续转弯产生影响
						Motor2.Sigma_MotorPulse = 0;
						Flag.Is_Track = 0;
						
						// 确定识别的数字，是在左边还是右边，2代表左边，3代表右边
						if(Flag.Is_Sure_NumThree == 2) {
							Flag.Is_Select_Left = 1;
						}
						else if(Flag.Is_Sure_NumThree == 3) {
							Flag.Is_Select_Right = 1;
						}
						else if(Flag.Is_Sure_NumFour == 2){      // 如果左边识别数字与开始相同，执行左转程序
							Flag.Is_Select_Left = 1;
						}
						else if(Flag.Is_Sure_NumFour == 3) {  // 如果右边识别数字与开始相同，执行左转程序
							Flag.Is_Select_Right = 1;
						}
						else if(Flag.Is_Sure_NumFive == 2) {
							Flag.Is_Select_Left = 1;
						}
						else if(Flag.Is_Sure_NumFive == 3) {
							Flag.Is_Select_Right = 1;
						}
						else if(Flag.Is_Sure_NumSix == 2) {
							Flag.Is_Select_Left = 1;
						}
						else if(Flag.Is_Sure_NumSix == 3) {
							Flag.Is_Select_Right = 1;
						}
						else if(Flag.Is_Sure_NumSeven == 2) {
							Flag.Is_Select_Left = 1;
						}
						else if(Flag.Is_Sure_NumSeven == 3) {
							Flag.Is_Select_Right = 1;
						}
						
						// 第二次识别对比结束
						
					}
				}
			}
			break; 
		default: break;
	}
}
