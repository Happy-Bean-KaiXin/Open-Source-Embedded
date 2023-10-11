#include "contorl.h"

PARAM_t Flag;   // ��־λ�ṹ��
LASER_t RED_LASER;  // ����ṹ��

float anglex, angley;                  // x�ᣬy��ĽǶ�ֵ
int RetangleX[4] = {21, 183, 184, 29};                    // ���x�������
int RetangleY[4] = {30, 32, 184, 189};                    // ���y�������

int Black_Retanx[4] = {77, 149, 149, 84};                                      // ��ɫ���ο�����x
int Black_Retany[4] = {47, 46, 94, };                                      // ��ɫ���ο�����y

int i;

// ��ʱ���ص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		
		static int cnt;
    if(htim->Instance == TIM4)//10ms
    {
			if(Flag.Is_Angle_Set) {
				// ʹ��PID����Ƕ�
				Flag.Anglex = Position_PID_RealizeX(Flag.X_AXIS, Flag.x_actual);
				Flag.Angley = Position_PID_RealizeY(Flag.Y_AXIS, Flag.y_actual);
				
				// ���PID
				Servo_Position.Servo_TarYout = Servo_Angle_Compare(Flag.Angley);  // ���ö��ת����Ŀ��ֵ
				Servo_Position.Servo_TarXout = Servo_Angle_Compare(Flag.Anglex);  // ���ö��ת����Ŀ��ֵ
				
				//���������         ���ת���̶��Ƕ���Ҫ�Ķ�ʱ���Ƚ�ֵ  ���ת���Ƕȵ�Ŀ��ֵ
				Init_CompareY += Servo_Position_PIDY(Init_CompareY, Servo_Position.Servo_TarYout);  // λ��ʽPID���� Init_CompareY��ǰ���ת����λ��
				angley = Servo_Compare_Angle(Init_CompareY);  // ��PID���������Ƚ�ֵת��Ϊ�Ƕ�ֵ
				
				Init_CompareX += Servo_Position_PIDX(Init_CompareX, Servo_Position.Servo_TarXout);  // λ��ʽPID1���� Init_CompareY��ǰ���ת����λ��
				anglex = Servo_Compare_Angle(Init_CompareX);  // ��PID���������Ƚ�ֵת��Ϊ�Ƕ�ֵ
				
				// �����Ƕȣ����ݿ�Ĵ�С����
				LIMIT(anglex, -90, 90);
				LIMIT(angley, -90, 90);
				
				Servo_X_Angle_Set(anglex);
				Servo_Y_Angle_Set(angley);    // ���ö��ת���ĽǶ�
				
				// �жϳ����Ƿ�����
				if(++cnt > 50) {
					cnt = 0;
					i++;
				}
			
			}
    }
		else if(htim->Instance == TIM3) {
			Flag.Is_10ms_YES = 1;
						// �жϳ����Ƿ�����
		}
}

// ���ڷ��ʹ�����
void print(UART_HandleTypeDef* huart, const char* buf, ...) {  // ���ں���
  const char *p = buf;
  char str[255] = {0};
  va_list v;
  va_start(v, buf);
  vsprintf(str, buf, v); //ʹ�ÿɱ�������ַ�����ӡ������sprintf
  if(HAL_UART_Transmit(huart, (uint8_t* )str, strlen(str), 1000) != HAL_OK) {
		Error_Handler();
	}
  va_end(v);
}

/*******************************************��ʾ������*******************************************/
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
	Flag.Angley = 0;   // ���ö���ĳ�ʼ��λ�Ƕ� PID�ջ�����
	Flag.x_centry = 120;
	Flag.y_centry = 98;
	Flag.FSTATE = Centy_To_Start;
	
}

// ����һ�κ�����б��
float calculateSlope(float x1, float y1, float x2, float y2) {
    return ((y2 - y1) / (x2 - x1));
}

// ����һ�κ����Ľؾ� ����slope��б��
int calculateIntercept(int x1, int y1, float slope) {
    return (y1 - slope * x1);
}

// ����һ�κ�����б�ʺͽؾ࣬���� y ֵ  intercept�ǽؾ�   ����x��ֵ����yֵ
int calculateY(int x, float slope, int intercept) {
    return slope * x + intercept;
}

// ����һ�κ�����б�ʺͽؾ࣬���� x ֵ  ����y����x��ֵ
int calculateX(float y, float slope, float intercept) {
    return (y - intercept) / slope;
}

int myabs(int p) {
	int q;
	q = p > 0? p : (-p);
	return q;
}
/******************************************END_OF_SHOW**************************************************/

/*******************************************���Ʋ㺯��*******************************************/
// �˶�Ŀ����ƴ�����
void Motion_TarCtrl(int* RetangleX, int* RetangleY) {
	switch(RED_LASER.Laser_State) {
		case Box_Square_State:
				RED_LASER.Laser_State = Any_Rectang_Box_State;
			
			break;
		case Any_Rectang_Box_State:   // ����λ�����״̬������㵽���յ㣨��һ����㣩����һ������б�ʣ��ؾ�
			/********************************** ״̬1 ***********************************************/
			if(Flag.FSTATE == Centy_To_Start) {  // ״̬1
					Flag.Slope = calculateSlope(Flag.x_centry, Flag.y_centry, RetangleX[0], RetangleY[0]);  // �����������꣬�����ⷽ���������� ����б��
					Flag.Intercpet = calculateIntercept(Flag.x_centry, Flag.y_centry, Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (RetangleX[0] - Flag.x_centry);
					Flag.dy = (RetangleY[0] - Flag.y_centry);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			/********************************** ״̬2 ***********************************************/
			else if(Flag.FSTATE == Start_To_Second) {  
					Flag.Slope = calculateSlope(RetangleX[0], RetangleY[0], RetangleX[1], RetangleY[1]);  // ����������꣬�յ����� ����б��
					Flag.Intercpet = calculateIntercept(RetangleX[0], RetangleY[0], Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (RetangleX[1] - RetangleX[0]);
					Flag.dy = (RetangleY[1] - RetangleY[0]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			/********************************** ״̬3 ***********************************************/
			else if(Flag.FSTATE == Second_To_Thrid) {  // ״̬3
					Flag.Slope = calculateSlope(RetangleX[1], RetangleY[1], RetangleX[2], RetangleY[2]);  // ����������꣬�յ����� ����б��
					Flag.Intercpet = calculateIntercept(RetangleX[1], RetangleY[1], Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (RetangleX[2] - RetangleX[1]);
					Flag.dy = (RetangleY[2] - RetangleY[1]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			/********************************** ״̬4 ***********************************************/
			else if(Flag.FSTATE == Thrid_To_Fourth) {  // ״̬4
					Flag.Slope = calculateSlope(RetangleX[2], RetangleY[2], RetangleX[3], RetangleY[3]);  // ����������꣬�յ����� ����б��
					Flag.Intercpet = calculateIntercept(RetangleX[2], RetangleY[2], Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (RetangleX[3] - RetangleX[2]);
					Flag.dy = (RetangleY[3] - RetangleY[2]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			/********************************** ״̬5 ***********************************************/
			else if(Flag.FSTATE == Fourth_To_End) {  // ״̬5
					Flag.Slope = calculateSlope(RetangleX[3], RetangleY[3], RetangleX[0], RetangleY[0]);  // ����������꣬�յ����� ����б��
					Flag.Intercpet = calculateIntercept(RetangleX[3], RetangleY[3], Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (RetangleX[0] - RetangleX[3]);
					Flag.dy = (RetangleY[0] - RetangleY[3]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			break;
		/********************************** �ȴ�״̬1-2-3-4-5������״̬ ***********************************************/
		case Cross_State:
			/********************************** ״̬1�ĵȴ�״̬ ***********************************************/
			if(Flag.FSTATE == Centy_To_Start) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = Flag.x_centry;  // ��ʵ��ֵy����y����������
					Flag.y_actual = Flag.x_centry;
					RED_LASER.Laser_State = Centry_X_Start_State;   // ��xΪ������y
				}
				else {                                            // ���dy����dx
					Flag.x_actual = Flag.x_centry;  // ��ʵ��ֵy����y����������
					Flag.y_actual = Flag.x_centry;
					RED_LASER.Laser_State = Centry_Y_Start_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			/********************************** ״̬2�ĵȴ�״̬ ***********************************************/
			else if(Flag.FSTATE == Start_To_Second) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = RetangleX[0];   // ��ʵ��ֵx  �����������x
					Flag.y_actual = RetangleY[0];   // ������ʼ����y

					RED_LASER.Laser_State = Start_X_Second_State;   // ��xΪ������y
				}
				else {                                            // ���dy����dx
					Flag.y_actual = RetangleY[0];   // 					
					Flag.x_actual = RetangleY[0];  
					RED_LASER.Laser_State = Start_Y_Second_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			/********************************** ״̬3�ĵȴ�״̬ ***********************************************/
			else if(Flag.FSTATE == Second_To_Thrid) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = RetangleX[1];   // ��ʵ��ֵx  ����ڶ�������x
					Flag.y_actual = RetangleY[1];   // ����ڶ�������y
					RED_LASER.Laser_State = Second_X_Thrid_State;   // ������һ��״̬
				}
				else {                                            // ���dy����dx
					Flag.x_actual = RetangleX[1];   // ��ʵ��ֵx  ����ڶ�������x
					Flag.y_actual = RetangleY[1];   // ����ڶ�������y
					RED_LASER.Laser_State = Second_Y_Thrid_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			/********************************** ״̬4�ĵȴ�״̬ ***********************************************/
			else if(Flag.FSTATE == Thrid_To_Fourth) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = RetangleX[2];   // ��ʵ��ֵx  �������������x
					Flag.y_actual = RetangleY[2];   // �������������y
					RED_LASER.Laser_State = Thrid_X_Fourth_State;   // ��xΪ������y
				}
				else {                                            // ���dy����dx
					Flag.x_actual = RetangleX[2];   // ��ʵ��ֵx  �������������x
					Flag.y_actual = RetangleY[2];   // �������������y
					RED_LASER.Laser_State = Thrid_Y_Fourth_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			/********************************** ״̬5�ĵȴ�״̬ ***********************************************/
			else if(Flag.FSTATE == Fourth_To_End) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = RetangleX[3];   // ��ʵ��ֵx  ������ĵ�����x
					Flag.y_actual = RetangleY[3];   // ������ĵ�����y
					RED_LASER.Laser_State = Fourth_X_End_State;   // ��xΪ������y
				}
				else {                                            // ���dy����dx
					Flag.x_actual = RetangleX[3];   // ��ʵ��ֵx  ������ĵ�����x
					Flag.y_actual = RetangleY[3];   // ������ĵ�����y
					RED_LASER.Laser_State = Fourth_Y_End_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			break;
		/********************************** ״̬1�µ���״̬ ***********************************************/
		case Centry_X_Start_State:        // ��xΪ������y��״̬  ���ĵ��������ʹ��x��������״̬
//			Flag.x_actual = Flag.x_centry;  // ��ʵ��ֵy����y����������
//			Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);  // ʹ��x��ʵ������ֵ��y
			
			if(Flag.dx > 0) {              // ���ֱ�߷��̵�dx>0 ����x��ʵ��ֵ�Ӽ�
				if(Flag.Is_10ms_YES == 1) {  // ���10ms���ˣ�����ʵ��ֵ��1
					
					Flag.x_actual+=7;
					Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);  // ʹ��x��ʵ������ֵ��y
					Flag.Is_10ms_YES = 0;
					if(myabs(Flag.x_actual - RetangleX[0]) < 4) {  // ���ʵ��ֵ���ڵ���ĵ�һ��Ŀ��ֵ��������һ��״̬
						Flag.y_actual = RetangleY[0];      // ��Y��ʵ��ֵǿ�е���Ŀ��ֵ
						Flag.x_actual = RetangleX[0];
						// ������һ��״̬
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
			}
			else {                         // ���dx<0 ����ʵ��ֵ����
				if(Flag.Is_10ms_YES == 1) {  // ����ʵ��ֵ��1
					
					Flag.x_actual-=7;
					Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet); 
					Flag.Is_10ms_YES = 0;
					if(myabs(Flag.x_actual - RetangleX[0]) < 4) {  // ���ʵ��ֵ���ڵ���ĵ�һ��Ŀ��ֵ��������һ��״̬
						Flag.y_actual = RetangleY[0];      // ��Y��ʵ��ֵǿ�е���Ŀ��ֵ
						Flag.x_actual = RetangleX[0];
						// ������һ��״̬
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
				
			}
			

			break;
		case Centry_Y_Start_State:   // ʹ��y��������״̬
//			Flag.y_actual = Flag.y_centry;   // ��ʵ��ֵy����y����������
//			Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);  // ����y��ʵ��ֵ��x
			
			if(Flag.dy > 0) {
				if(Flag.Is_10ms_YES == 1) {
					Flag.Is_10ms_YES = 0;
					Flag.y_actual+=7;
					if(myabs(Flag.y_actual - RetangleY[0]) < 4) {
						Flag.x_actual = RetangleX[0];
						Flag.y_actual = RetangleY[0];
						// ������һ��״̬
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
						// ������һ��״̬
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}				
			}
			

			break;
			/********************************** ״̬2�µ���״̬ ***********************************************/
			case Start_X_Second_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[1]) < 4) {
							Flag.y_actual = RetangleY[1];
							Flag.x_actual = RetangleX[1];
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				
				break;
			/********************************** ״̬3�µ���״̬ ***********************************************/
			case Second_X_Thrid_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[2]) < 4) {
							Flag.y_actual = RetangleY[2];
							Flag.x_actual = RetangleX[2];
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				break;
			/********************************** ״̬4�µ���״̬ ***********************************************/
			case Thrid_X_Fourth_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=7;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - RetangleX[3]) < 10) {
							Flag.y_actual = RetangleY[3];
							Flag.x_actual = RetangleX[3];
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				break;
			/********************************** ״̬5�µ���״̬ ***********************************************/
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
							// ������һ��״̬
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
							// ������һ��״̬
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
		case Any_Rectang_Box_State:   // ����λ�����״̬������㵽���յ㣨��һ����㣩����һ������б�ʣ��ؾ�
			/********************************** ״̬1 ***********************************************/
			if(Flag.FSTATE == Centy_To_Start) {  // ״̬1
					Flag.Slope = calculateSlope(Flag.x_centry, Flag.y_centry, Black_Retanx[0], Black_Retany[0]);  // �����������꣬�����ⷽ���������� ����б��
					Flag.Intercpet = calculateIntercept(Flag.x_centry, Flag.y_centry, Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (Black_Retanx[0] - Flag.x_centry);
					Flag.dy = (Black_Retany[0] - Flag.y_centry);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			/********************************** ״̬2 ***********************************************/
			else if(Flag.FSTATE == Start_To_Second) {  
					Flag.Slope = calculateSlope(Black_Retanx[0], Black_Retany[0], Black_Retanx[1], Black_Retany[1]);  // ����������꣬�յ����� ����б��
					Flag.Intercpet = calculateIntercept(Black_Retanx[0], Black_Retany[0], Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (Black_Retanx[1] - Black_Retanx[0]);
					Flag.dy = (Black_Retany[1] - Black_Retany[0]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			/********************************** ״̬3 ***********************************************/
			else if(Flag.FSTATE == Second_To_Thrid) {  // ״̬3
					Flag.Slope = calculateSlope(Black_Retanx[1], Black_Retany[1], Black_Retanx[2], Black_Retany[2]);  // ����������꣬�յ����� ����б��
					Flag.Intercpet = calculateIntercept(Black_Retanx[1], Black_Retany[1], Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (Black_Retanx[2] - Black_Retanx[1]);
					Flag.dy = (Black_Retany[2] - Black_Retany[1]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			/********************************** ״̬4 ***********************************************/
			else if(Flag.FSTATE == Thrid_To_Fourth) {  // ״̬4
					Flag.Slope = calculateSlope(Black_Retanx[2], Black_Retany[2], Black_Retanx[3], Black_Retany[3]);  // ����������꣬�յ����� ����б��
					Flag.Intercpet = calculateIntercept(Black_Retanx[2], Black_Retany[2], Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (Black_Retanx[3] - Black_Retanx[2]);
					Flag.dy = (Black_Retany[3] - Black_Retany[2]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			/********************************** ״̬5 ***********************************************/
			else if(Flag.FSTATE == Fourth_To_End) {  // ״̬5
					Flag.Slope = calculateSlope(Black_Retanx[3], Black_Retany[3], Black_Retanx[0], Black_Retany[0]);  // ����������꣬�յ����� ����б��
					Flag.Intercpet = calculateIntercept(Black_Retanx[3], Black_Retany[3], Flag.Slope);          // ����ؾ�
					// ����dx�� dy
					Flag.dx = (Black_Retanx[0] - Black_Retanx[3]);
					Flag.dy = (Black_Retany[0] - Black_Retany[3]);
					if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
						RED_LASER.Laser_State = Cross_State;   // ��xΪ������y
					}
					else {                                            // ���dy����dx
						RED_LASER.Laser_State = Cross_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
					}
			}
			break;
		/********************************** �ȴ�״̬1-2-3-4-5������״̬ ***********************************************/
		case Cross_State:
			/********************************** ״̬1�ĵȴ�״̬ ***********************************************/
			if(Flag.FSTATE == Centy_To_Start) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = Flag.x_centry;  // ��ʵ��ֵy����y����������
					Flag.y_actual = Flag.x_centry;
					RED_LASER.Laser_State = Centry_X_Start_State;   // ��xΪ������y
				}
				else {                                            // ���dy����dx
					Flag.x_actual = Flag.x_centry;  // ��ʵ��ֵy����y����������
					Flag.y_actual = Flag.x_centry;
					RED_LASER.Laser_State = Centry_Y_Start_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			/********************************** ״̬2�ĵȴ�״̬ ***********************************************/
			else if(Flag.FSTATE == Start_To_Second) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = Black_Retanx[0];   // ��ʵ��ֵx  �����������x
					Flag.y_actual = Black_Retany[0];   // ������ʼ����y

					RED_LASER.Laser_State = Start_X_Second_State;   // ��xΪ������y
				}
				else {                                            // ���dy����dx
					Flag.y_actual = Black_Retanx[0];   // 					
					Flag.x_actual = Black_Retany[0];  
					RED_LASER.Laser_State = Start_Y_Second_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			/********************************** ״̬3�ĵȴ�״̬ ***********************************************/
			else if(Flag.FSTATE == Second_To_Thrid) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = Black_Retanx[1];   // ��ʵ��ֵx  ����ڶ�������x
					Flag.y_actual = Black_Retany[1];   // ����ڶ�������y
					RED_LASER.Laser_State = Second_X_Thrid_State;   // ������һ��״̬
				}
				else {                                            // ���dy����dx
					Flag.x_actual = Black_Retanx[1];   // ��ʵ��ֵx  ����ڶ�������x
					Flag.y_actual = Black_Retany[1];   // ����ڶ�������y
					RED_LASER.Laser_State = Second_Y_Thrid_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			/********************************** ״̬4�ĵȴ�״̬ ***********************************************/
			else if(Flag.FSTATE == Thrid_To_Fourth) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = Black_Retanx[2];   // ��ʵ��ֵx  �������������x
					Flag.y_actual = Black_Retany[2];   // �������������y
					RED_LASER.Laser_State = Thrid_X_Fourth_State;   // ��xΪ������y
				}
				else {                                            // ���dy����dx
					Flag.x_actual = Black_Retanx[2];   // ��ʵ��ֵx  �������������x
					Flag.y_actual = Black_Retany[2];   // �������������y
					RED_LASER.Laser_State = Thrid_Y_Fourth_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			/********************************** ״̬5�ĵȴ�״̬ ***********************************************/
			else if(Flag.FSTATE == Fourth_To_End) {
				if(myabs(Flag.dx) - myabs(Flag.dy) > 0) {         // �Ƚ�dx��dy�Ĵ�Сȥ��x��y˭�Ǳ���
					Flag.x_actual = Black_Retanx[3];   // ��ʵ��ֵx  ������ĵ�����x
					Flag.y_actual = Black_Retany[3];   // ������ĵ�����y
					RED_LASER.Laser_State = Fourth_X_End_State;   // ��xΪ������y
				}
				else {                                            // ���dy����dx
					Flag.x_actual = Black_Retanx[3];   // ��ʵ��ֵx  ������ĵ�����x
					Flag.y_actual = Black_Retany[3];   // ������ĵ�����y
					RED_LASER.Laser_State = Fourth_Y_End_State;   // ����yΪ������x��������y������x��״̬�����ĵ��������ʹ��y��������״̬��
				}
			}
			break;
		/********************************** ״̬1�µ���״̬ ***********************************************/
		case Centry_X_Start_State:        // ��xΪ������y��״̬  ���ĵ��������ʹ��x��������״̬
//			Flag.x_actual = Flag.x_centry;  // ��ʵ��ֵy����y����������
//			Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);  // ʹ��x��ʵ������ֵ��y
			
			if(Flag.dx > 0) {              // ���ֱ�߷��̵�dx>0 ����x��ʵ��ֵ�Ӽ�
				if(Flag.Is_10ms_YES == 1) {  // ���10ms���ˣ�����ʵ��ֵ��1
					
					Flag.x_actual+=1;
					Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);  // ʹ��x��ʵ������ֵ��y
					Flag.Is_10ms_YES = 0;
					if(myabs(Flag.x_actual - Black_Retanx[0]) < 2) {  // ���ʵ��ֵ���ڵ���ĵ�һ��Ŀ��ֵ��������һ��״̬
						Flag.y_actual = Black_Retany[0];      // ��Y��ʵ��ֵǿ�е���Ŀ��ֵ
						Flag.x_actual = Black_Retanx[0];
						// ������һ��״̬
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
			}
			else {                         // ���dx<0 ����ʵ��ֵ����
				if(Flag.Is_10ms_YES == 1) {  // ����ʵ��ֵ��1
					
					Flag.x_actual-=1;
					Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet); 
					Flag.Is_10ms_YES = 0;
					if(myabs(Flag.x_actual - Black_Retanx[0]) < 2) {  // ���ʵ��ֵ���ڵ���ĵ�һ��Ŀ��ֵ��������һ��״̬
						Flag.y_actual = Black_Retany[0];      // ��Y��ʵ��ֵǿ�е���Ŀ��ֵ
						Flag.x_actual = Black_Retanx[0];
						// ������һ��״̬
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}
				
			}
			

			break;
		case Centry_Y_Start_State:   // ʹ��y��������״̬
//			Flag.y_actual = Flag.y_centry;   // ��ʵ��ֵy����y����������
//			Flag.x_actual = calculateX(Flag.y_actual, Flag.Slope, Flag.Intercpet);  // ����y��ʵ��ֵ��x
			
			if(Flag.dy > 0) {
				if(Flag.Is_10ms_YES == 1) {
					Flag.Is_10ms_YES = 0;
					Flag.y_actual+=1;
					if(myabs(Flag.y_actual - Black_Retany[0]) < 2) {
						Flag.x_actual = Black_Retanx[0];
						Flag.y_actual = Black_Retany[0];
						// ������һ��״̬
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
						// ������һ��״̬
						Flag.FSTATE = Start_To_Second;
						RED_LASER.Laser_State = Any_Rectang_Box_State;
					}
				}				
			}
			

			break;
			/********************************** ״̬2�µ���״̬ ***********************************************/
			case Start_X_Second_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[1]) < 2) {
							Flag.y_actual = Black_Retany[1];
							Flag.x_actual = Black_Retanx[1];
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
							Flag.FSTATE = Second_To_Thrid;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				
				break;
			/********************************** ״̬3�µ���״̬ ***********************************************/
			case Second_X_Thrid_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[2]) < 2) {
							Flag.y_actual = Black_Retany[2];
							Flag.x_actual = Black_Retanx[2];
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
							Flag.FSTATE = Thrid_To_Fourth;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				break;
			/********************************** ״̬4�µ���״̬ ***********************************************/
			case Thrid_X_Fourth_State:
				if(Flag.dx > 0) {
					if(Flag.Is_10ms_YES == 1) {
						Flag.Is_10ms_YES = 0;
						Flag.x_actual+=1;
						Flag.y_actual = calculateY(Flag.x_actual, Flag.Slope, Flag.Intercpet);
						if(myabs(Flag.x_actual - Black_Retanx[3]) < 2) {
							Flag.y_actual = Black_Retany[3];
							Flag.x_actual = Black_Retanx[3];
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
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
							// ������һ��״̬
							Flag.FSTATE = Fourth_To_End;
							RED_LASER.Laser_State = Any_Rectang_Box_State;
						}
					}				
				}
				break;
			/********************************** ״̬5�µ���״̬ ***********************************************/
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
							// ������һ��״̬
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
							// ������һ��״̬
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