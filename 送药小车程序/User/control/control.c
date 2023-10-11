#include "control.h"
#include "stdarg.h"
#include "string.h"
#include "dma.h"
#include "math.h"

PARAM_t Flag;

float WheelOneCircleDis=WheelDiameter*pi;//����С����������һȦ�������

int Rpm_Encoder_Pulse_MAX;

// ��ʱ���жϻص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	
	if(htim->Instance == TIM4){
		
		//1.��λʱ��������
		Motor1.Uinttime_MotorPulse = GetMotorPulse(1);
		Motor2.Uinttime_MotorPulse = GetMotorPulse(3);

	  //����ȡ���ĵ�λʱ���ڵ�������תΪ��λΪrpm
		Motor1.speed = (float)(Motor1.Uinttime_MotorPulse/(52.0*30.0*10.0))*1000.0*60.0;
		Motor2.speed = (float)(Motor2.Uinttime_MotorPulse/(52.0*30.0*10.0))*1000.0*60.0;
		//2.�ۼ�������
		Motor1.Sigma_MotorPulse += Motor1.Uinttime_MotorPulse;
		Motor2.Sigma_MotorPulse += Motor2.Uinttime_MotorPulse;
		
		
		if(Flag.Is_Location_Speed == 1) {
			
			//3.λ�û�
			Motor1.Target_Position = Nlaps_Encoder_Cnt(Motor1.Num);
			Motor2.Target_Position = Nlaps_Encoder_Cnt(Motor2.Num);
			
			Location.Motor1_Out = Location_PID_Realize(Motor1.Sigma_MotorPulse, Motor1.Target_Position);
			Location.Motor2_Out = Location_PID_Realize(Motor2.Sigma_MotorPulse, Motor2.Target_Position);
			
			//�޷�ֵ220��
			//λ�û��޷�
			if(Location.Motor1_Out>MOTOR_SPEED_MAX)Location.Motor1_Out = MOTOR_SPEED_MAX;
			else if(Location.Motor1_Out<-MOTOR_SPEED_MAX)Location.Motor1_Out = -MOTOR_SPEED_MAX;
			
			if(Location.Motor2_Out>MOTOR_SPEED_MAX)Location.Motor2_Out = MOTOR_SPEED_MAX;
			else if(Location.Motor2_Out<-MOTOR_SPEED_MAX)Location.Motor2_Out = -MOTOR_SPEED_MAX;

			// �ٶȻ�  �˴����������ٶȻ�������������һ�δ���ֵ��Ӱ����һ�ε��µĴ���ֵ��Ϊ����Ӱ�죬��˶��������ٶȻ�����
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
  vsprintf(str, buf, v); //ʹ�ÿɱ�������ַ�����ӡ������sprintf
//	HAL_UART_Transmit(huart, (uint8_t* )str, strlen(str), 1000); 
	if(HAL_UART_Transmit_DMA(huart, (uint8_t* )str, strlen(str)) != HAL_OK) {
		Error_Handler(); 
	}		// DMAģʽ����
	
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

/*****************************************С��ѭ���ײ㺯��************************************************************/

/**
 *@brief:����ѭ��������pid����С��ת��ʹС�����ں����м�
 * @param:
 *        [in]int TraceDate: ѭ�������������ֵ
 * @return: ���ص��ڵ���ٶȵ�ת��pwm
 */
int ChangeTraceTurn(int TraceDate)
{
	int pwm=0;
	int bias;
	bias = TraceDate;
	pwm = Turn_PID_Realize( bias);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//�޷�
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:����pid������ߵ����Ŀ���ٶ�
 * @param:
 *        [in]int EncodeSpdL: ��ǰ��������������ֵ
 *        [in]float TarSpdL:��ߵ��Ŀ���ٶ�,����ٶ�Խ1.19m/s
 * @return: ������ߵ��������pwmռ�ձ�
 */
int ChangeSpeedMotorL(int NowEncodeSpdL,float TarSpdL)
{
	int pwm=0;
	int TarEncodeSpdL;
	TarEncodeSpdL=(int)((TarSpdL*ACircleEncoder)/(WheelOneCircleDis*100));//����Ŀ���ٶ����Ŀ��������ٶ�

	pwm=Position_PID_Realize(NowEncodeSpdL, TarEncodeSpdL);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//�޷�
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:����pid�����ұߵ����Ŀ���ٶ�
 * @param:
 *        [in]int NowEncodeSpdR: ��ǰ�ҵ������������ֵ
 *        [in]float TarSpdR:�ұߵ��Ŀ���ٶ�,�����޷�����ٶ�ԼΪ0.7m/s
 * @return: �����ұߵ��������pwmռ�ձ�
 */
int ChangeSpeedMotorR(int NowEncodeSpdR,float TarSpdR)
{
	int pwm=0;
	int TarEncodeSpdR;
	TarEncodeSpdR=(int)((TarSpdR*ACircleEncoder)/(WheelOneCircleDis*100));//����Ŀ���ٶ����Ŀ��������ٶ�
	
	pwm=Position_PID_Realize(NowEncodeSpdR, TarEncodeSpdR);
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//�޷�
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	return pwm;
}

/**
 *@brief: ��С������ѭ��������
 *@param:
 *        [in]TraceDate: ѭ�������������ֵ
 *        [in]TarSpeed:ѭ����Ŀ���ٶ�,�ٶȴ�С0-0.7m/s
 *@return: ����Ŀ��㷵��1�����򷵻�0
 */
void TraceMove(int TraceDate,float TarSpeed)
{
	int turnpwm=0;
	int spdpwml=0,spdpwmr=0;
	int pwml=0,pwmr=0;
	
	turnpwm=ChangeTraceTurn(TraceDate);//ת��PID
	
	spdpwml=ChangeSpeedMotorL(Motor2.Uinttime_MotorPulse, TarSpeed);
	spdpwmr=ChangeSpeedMotorR(Motor1.Uinttime_MotorPulse, TarSpeed);

//	printf("Encode_Left:%d Encode_Right:%d\r\n",Encode_Left,Encode_Right);
	
	pwmr=turnpwm+spdpwmr;
	if(pwmr>MAX_MOTOR_PWM)		pwmr=MAX_MOTOR_PWM;//�޷�
	else if(pwmr<-MAX_MOTOR_PWM)  pwmr=-MAX_MOTOR_PWM;
	
	pwml=-turnpwm+spdpwml;
	if(pwml>MAX_MOTOR_PWM)		pwml=MAX_MOTOR_PWM;//�޷�
	else if(pwml<-MAX_MOTOR_PWM)  pwml=-MAX_MOTOR_PWM;
	
	Set_PWM(pwml, pwmr);
}

/*@brief:����С����Ŀ�����pid������ߵ���ٶ�
 * @param:
 *        [in]int NowEncodeLALL: ��ǰ���������������
 *        [in]float TarDis:��ߵ��Ŀ����룬0-10M
 * @return: ������ߵ��������pwmռ�ձ�
 */
int ChangeDisSpeedMotorL(int NowEncodeLALL,float TarDis)
{
	int pwm=0;
	int TarEncodeLALL;
	TarEncodeLALL=(int)(((TarDis/WheelOneCircleDis)*ACircleEncoder));//����Ŀ��������Ŀ�������ֵ
	pwm = Position_PID_Realize1(NowEncodeLALL, TarEncodeLALL);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//�޷�
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:����С����Ŀ�����pid�����ұߵ���ٶ�
 * @param:
 *        [in]int NowEncodeRALL: ��ǰ�ҵ�������������
 *        [in]float TarDis:��ߵ��Ŀ����룬0-10M
 * @return: �����ұߵ��������pwmռ�ձ�
 */
int ChangeDisSpeedMotorR(int NowEncodeRALL,float TarDis)
{
	int pwm=0;
	int TarEncodeRALL;
	TarEncodeRALL=(int)(((TarDis/WheelOneCircleDis)*ACircleEncoder));//����Ŀ��������Ŀ�������ֵ
	pwm = Position_PID_Realize1(NowEncodeRALL, TarEncodeRALL);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//�޷�
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief: ����Ŀ�����ѭ����ʻ������ʻĿ������ֹͣѭ��
 *@param:
 *        [in]TraceDate: ѭ�������������ֵ
 *        [in]TarDis:С����ʻ��Ŀ�����,�����Χ�ݶ�
 *@return: ����Ŀ��㷵��1�����򷵻�0
 */
void TraceMoveWithDis(int TraceDate,float TarDis)
{
	int turnpwm=0;
	int spdlpwm,spdrpwm;
	int pwml=0,pwmr=0;
	int TarEncodeRALL=(int)(((TarDis/WheelOneCircleDis)*ACircleEncoder)); // ����Ŀ��������Ŀ�������ֵ
	
	turnpwm=ChangeTraceTurn(TraceDate);
	
	spdlpwm = ChangeDisSpeedMotorL(Motor1.Sigma_MotorPulse, TarDis);
	spdrpwm = ChangeDisSpeedMotorR(Motor2.Sigma_MotorPulse, TarDis);
		
	pwmr=turnpwm+spdrpwm;
	
	if(pwmr>MAX_MOTOR_PWM)		pwmr=MAX_MOTOR_PWM;//�޷�
	else if(pwmr<-MAX_MOTOR_PWM)  pwmr=-MAX_MOTOR_PWM;
	pwml=-turnpwm+spdlpwm;
	if(pwml>MAX_MOTOR_PWM)		pwml=MAX_MOTOR_PWM;//�޷�
	else if(pwml<-MAX_MOTOR_PWM)  pwml=-MAX_MOTOR_PWM;
	
	Set_PWM(pwml, pwmr);
	
	if((gfp_abs(TarEncodeRALL - ((int)(Motor1.Sigma_MotorPulse + Motor2.Sigma_MotorPulse)/2)) < 1000)) {
			
			Motor_Stop();
			Flag.Is_DisArrive = 1;  // ͣ����־λ
		}
}

/*@brief:ͨ��PD����������С������һ���ľ���
 * @param:
 *        [in]float NowEncodeLALL: ��ǰ���ֱ����������
 *        [in]float NowEncodeRALL: ��ǰ���ֱ����������
 *        [in]float TarDisL: ��ߵ��Ŀ����룬0-10M
 *        [in]float TarDisR:�ұߵ��Ŀ����룬0-10M
 *@return: ���߽�������0�����߷���1
 */
void MoveDis(int NowEncodeLALL,int NowEncodeRALL,float TarDisL,float TarDisR)
{
	int dislpwm,disrpwm;
	
	dislpwm=ChangeDisSpeedMotorL(NowEncodeLALL,TarDisL);
	disrpwm=ChangeDisSpeedMotorR(NowEncodeRALL,TarDisR);
	
	if(gfp_abs(dislpwm)<300) dislpwm=0;//��ֹ���ת�����պĵ�
	if(gfp_abs(disrpwm)<300) disrpwm=0;
	
	Set_PWM(dislpwm, disrpwm);
}

/************************************End of File***********************************************/

/**********************************С������Ӧ�ò�***********************************************/
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

	Flag.Is_Location_Speed = 0;   // �����ٶ�λ��PID�Ƿ�����־λ
	Flag.Is_Track = 0;
	Flag.OLED_FACE = 4;           // OLED��ʾ����
	Flag.Is_Medicine = 0;         // ��ʼ��ҩƷ
	Flag.Is_DisArrive = 0;        // 0��ʾ��ʼ����δ����
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

#define Follow_Dis 0.85      // ����ֱ�о���
#define Turn_Dis   0.48      // ���ؾ���
#define Follow_MID_Dis 1.445  // �ж�ֱ�о���   ���뵽�˽���ʶ��
#define Follow_FAR_Dis 2.65  // Զ��ֱ�о���  ����
#define Follow_Keep_Dis 0.38  // 

extern int TraceDate;
CAR_t CAR;

void Car_State_Machine(void) {
	switch(CAR.Car_State) {
		case Test_Medicine_State:          // ���ҩƷ״̬
				if(Flag.Is_Medicine == 1) {    // �����⵽��ҩƷ
					Flag.Is_Track = 1;
					if(Flag.Is_NumOne == 1) {    // ���ʶ��������1
						Flag.Is_NumOne = 0;
						Flag.Is_Select_Left = 1;   // ������ת���־
						CAR.Car_Dis = Follow_Dis;  // ����ֱ�о���
						CAR.Car_State = Follow_Line_State;  // ����ֱ��״̬
					}
					else if(Flag.Is_NumTwo == 1) {  // ���ʶ��������2
						Flag.Is_NumTwo = 0;
						Flag.Is_Select_Right = 1;
						CAR.Car_Dis = Follow_Dis;
						CAR.Car_State = Follow_Line_State;
					}
					// ��Զ������
					else if(Flag.Is_DIs_MID_FAR == 1) {   // ���ʶ������Զ������
						CAR.Car_Dis = Follow_MID_Dis;
						Flag.Is_Mid_Dis = 1;    // ʶ����Զ�����֣����ж˷��ؾ����־��Ϊ1
						
						CAR.Car_State = Follow_Line_State;
						if(Flag.Is_NumThree == 1) {  // ���ʶ��������3
							Flag.Is_NumThree = 0;
							Flag.Is_Sure_NumThree = 1;  // ��������ʶ��������ٴ�ʶ�𣬽�����־��Ϊ1�����Ҳ��ٴ�ʶ�𣬽�����־λ��Ϊ2
						}
						else if(Flag.Is_NumFour == 1) {  // ���ʶ��������4
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
//						Flag.Is_Num_Recongnize = 0;  // �ر�����ʶ��   ��ʼ���Բ��ùر�����ʶ��
					}
				}

			break;
		case Follow_Line_State:     // ǰ��״̬
			if(Flag.Is_Track == 1) {
				if(TraceDate == 30000) {
					Motor_Stop();
				}
				else {
					TraceMoveWithDis(TraceDate, CAR.Car_Dis);
					if(Flag.Is_DisArrive == 1) {  // ���뵽��
						CAR.Car_State = Cross_State;
						Motor1.Sigma_MotorPulse = 0;  // �������Ժ���ת�����Ӱ��
						Motor2.Sigma_MotorPulse = 0;
						Flag.Is_Track = 0;
						Flag.Is_NumOne = 0;  // ������1�ı�־λ����
						Flag.Is_NumTwo = 0;  // ��������2��־λ
						
						if(Flag.Is_Medicine == 1) {  //���ҩƷ�ڳ��ϣ��ҵ�����ҩ�������������
							if(fabs(CAR.Car_Dis - Turn_Dis) < 0.1) 
								Red_Set;           // ������ɫС��
						}
					}
				}
			}
			
			break;
		case Cross_State:       // �ȴ�״̬
			Flag.Is_DisArrive = 0;
			if(Flag.Is_Select_Left == 1) {
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Select_Left = 0;      // ������ת���־λ
					// ��תָ��
				CAR.Car_State = Select_Left_State;  // ������ת��״̬
				Flag.Is_Location_Speed = 1;         // ����λ���ٶȻ�PID
			}
			else if(Flag.Is_Select_Right == 1) {
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Select_Right = 0;     // ������ת���־λ
					// ��תָ��
				CAR.Car_State = Select_Right_State;  // ������ת��״̬
				Flag.Is_Location_Speed = 1;
			}
			else if(Flag.Is_Medicine == 0) {   // ҩƷ��ȡ��
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Location_Speed = 1;      // ����λ���ٶȱջ�����
				
				if(Flag.Is_Right_Turn)           // ��ʼ������ת�䣬�ٻ�����·�Ͻ�������ת�䣬�����ͣ��״̬
					CAR.Car_State = Stop_State;
				else if(Flag.Is_Left_Turn)
					CAR.Car_State = Stop_State;
				else 
					CAR.Car_State = Turn_Around_State;
			}
			else if(Flag.Is_DIs_MID_FAR == 1) {     // ���ֱ�е��ˣ���Զ�˵�һ��ʶ��·��
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Location_Speed = 1;
				CAR.Car_State = Select_MID_Left_State;
				Flag.Is_Num_Recongnize = 1;   // ��������ʶ��
			}
			else if(Flag.Is_Car_Stop == 1) {
				Flag.Is_DIs_MID_FAR = 0;
				Flag.Is_Location_Speed = 0;
				CAR.Car_State = Stop_State;
			}
			break;
		case Select_Left_State:  // ��ת��״̬
			if(Flag.Is_Location_Speed == 1) {
				Motor1.Num = Car_Turn(-90);
				Motor2.Num = Car_Turn(90);  
				HAL_Delay(500);
				Flag.Is_Left_Log = 1;   // ��ת����ɺ󣬼�¼һ��
//				if(gfp_abs(Get_Adc(0) - Get_Adc(4)) < 550) {
					Flag.Is_Location_Speed = 0;      // �ر��ٶ�λ�û�PID
					Flag.Is_Track = 1;               // ����ѭ����־λ
					CAR.Car_State = Follow_Line_State;
					Motor1.Sigma_MotorPulse = 0;
					Motor2.Sigma_MotorPulse = 0;
					Motor1.Num = Motor2.Num = 0;
					
					if(Flag.Is_Left_Turn == 1) {
						if(Flag.Is_Mid_Dis) {  // ������ж˷���
							Flag.Is_Mid_Dis  = 0;
							CAR.Car_Dis = Follow_MID_Dis + Follow_Keep_Dis - 0.21;
						}
						else
							CAR.Car_Dis = Follow_Dis;
						
						Flag.Is_Car_Stop = 1;
					}
					else {
						CAR.Car_Dis = Turn_Dis;   // ���жˣ�ת��180�����ʻ0.48m
					}
//				}
			}

			break;
		case Select_Right_State:        // ��ת��״̬
			if(Flag.Is_Location_Speed == 1) {
				Motor1.Num = Car_Turn(90);
				Motor2.Num = Car_Turn(-90);
				HAL_Delay(500);
				Flag.Is_Right_Log = 1;   // ��ת����ɺ󣬼�¼һ��
				
				Flag.Is_Location_Speed = 0;      // �ر��ٶ�λ�û�PID
				Flag.Is_Track = 1;               // ����ѭ����־λ
				CAR.Car_State = Follow_Line_State;
				Motor1.Sigma_MotorPulse = 0;
				Motor2.Sigma_MotorPulse = 0;
				Motor1.Num = Motor2.Num = 0;
				
				if(Flag.Is_Right_Turn == 1) {    // �ڷ���·�̽�����ת��
					
					if(Flag.Is_Mid_Dis) {  // �ѽ��   ������ж�
						Flag.Is_Mid_Dis = 0;
						CAR.Car_Dis = Follow_MID_Dis + Follow_Keep_Dis - 0.21;  // �����ж˷��ؾ���
					}
					else
						CAR.Car_Dis = Follow_Dis;    // �������ý��˷��ؾ���
					
//					CAR.Car_Dis = Follow_Dis;      // ������ת�󣬷���·�̵�ֱ�о���
					Flag.Is_Car_Stop = 1;          // ������ʱû���õ������־�� ��ͣ����־��Ϊ1
				}
				else {
					CAR.Car_Dis = Turn_Dis;        // ������������·�̵���ת�����
				}
			}
			break;
		case Turn_Around_State:        // ��ͷת��״̬
			Motor1.Num = Car_Turn(180);
			Motor2.Num = Car_Turn(-180);
			HAL_Delay(700);
			if(gfp_abs(Get_Adc(8) - Get_Adc(4)) < 550) {
				Flag.Is_Location_Speed = 0;
				Flag.Is_Track = 1;
				CAR.Car_State = Follow_Line_State;     // ����ֱ��״̬
				CAR.Car_Dis = Turn_Dis;
				Motor1.Sigma_MotorPulse = 0;
				Motor2.Sigma_MotorPulse = 0;
				Motor1.Num = Motor2.Num = 0;
				
				if(Flag.Is_Left_Log == 1) {   // ���������һ����ת�䣬���ȥ��·�Ͻ�������ת��
					Flag.Is_Left_Log = 0;       // ����¼��ת���־����
					Flag.Is_Select_Right = 1;   // ������ת���־
					Flag.Is_Right_Turn = 1;     // ������·�̵���ת���־��Ϊ1
				}
				else if(Flag.Is_Right_Log == 1) {  // ���������һ����ת�䣬���ȥ��·�Ͻ�������ת��
					Flag.Is_Right_Log = 0;
					Flag.Is_Select_Left = 1;
					Flag.Is_Left_Turn = 1;
				}		
			}
			break;
		case Stop_State:    // ͣ��״̬
			Flag.Is_Location_Speed = 0;     // �ر��ٶ�λ�û�
			Flag.Is_Track = 0;              // �ر�ѭ������
			Motor_Stop();                   // ����ͣ��
			Green_Set;      // �ص���㣬�����̵�
			break;
		case Select_MID_Left_State:      // �Ƚ�����תʶ��
			Flag.Is_Num_Recongnize = 1;
			Motor1.Num = Car_Turn(-15);
			Motor2.Num = Car_Turn(15);
			HAL_Delay(2000);
			// ����ٴ�ʶ����Ŀ��
			if(Flag.Is_Sure_NumThree == 1)
				while(Flag.Is_NumThree == 1) {
					
					Flag.Is_Sure_NumThree = 2;   // ��תʶ�𣬽��ٴ�ʶ���Ŀ����Ϊ2
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
		case Select_MID_Right_State:     // �ڽ�����תʶ��
			Flag.Is_Num_Recongnize = 1; 
			Motor1.Num = Car_Turn(35);
			Motor2.Num = Car_Turn(-35);
			HAL_Delay(2000);   // ��ʱ���������ڴ��ڻش����ݱȽ���
			// ����ٴ�ʶ����Ŀ��
			if(Flag.Is_Sure_NumThree == 1)
				while(Flag.Is_NumThree == 1) {
					
					Flag.Is_Sure_NumThree = 3;    // ��תʶ�� ���ٴ�ʶ���Ŀ����Ϊ3
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
			CAR.Car_State = Turn_Follow_State;  // ���복ͷ����״̬
			Motor1.Sigma_MotorPulse = 0;
			Motor2.Sigma_MotorPulse = 0;
			Motor1.Num = Motor2.Num = 0;
			
			break;
		case Turn_Follow_State:    // ����ͷת����
			Motor1.Num = Car_Turn(-20);
			Motor2.Num = Car_Turn(20);
			HAL_Delay(700);
			Motor1.Sigma_MotorPulse = 0;
			Motor2.Sigma_MotorPulse = 0;
			Motor1.Num = Motor2.Num = 0;
			Flag.Is_Num_Recongnize = 0;     // �ر�����ʶ��
			Flag.Is_Location_Speed = 0;  // �ر��ٶ�λ�û�
			Flag.Is_Track = 1;           // ����ѭ��
			CAR.Car_State = Keep_Fllow_Straight_State;  // �����������ִ�е�״̬
			CAR.Car_Dis = Follow_Keep_Dis;              // ���ü�����ʻ�ľ���
			break;
		case Keep_Fllow_Straight_State:  // ��ͷת��������󣬱��ּ���ֱ�е�״̬
			
			if(Flag.Is_Track == 1) {
				if(TraceDate == 30000) {
					Motor_Stop();
				}
				else {
					TraceMoveWithDis(TraceDate, CAR.Car_Dis);
					Flag.Is_DIs_MID_FAR = 0;
					if(Flag.Is_DisArrive == 1) {  // ���뵽��
						CAR.Car_State = Cross_State;
						Motor1.Sigma_MotorPulse = 0;  // �������Ժ���ת�����Ӱ��
						Motor2.Sigma_MotorPulse = 0;
						Flag.Is_Track = 0;
						
						// ȷ��ʶ������֣�������߻����ұߣ�2������ߣ�3�����ұ�
						if(Flag.Is_Sure_NumThree == 2) {
							Flag.Is_Select_Left = 1;
						}
						else if(Flag.Is_Sure_NumThree == 3) {
							Flag.Is_Select_Right = 1;
						}
						else if(Flag.Is_Sure_NumFour == 2){      // ������ʶ�������뿪ʼ��ͬ��ִ����ת����
							Flag.Is_Select_Left = 1;
						}
						else if(Flag.Is_Sure_NumFour == 3) {  // ����ұ�ʶ�������뿪ʼ��ͬ��ִ����ת����
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
						
						// �ڶ���ʶ��ԱȽ���
						
					}
				}
			}
			break; 
		default: break;
	}
}
